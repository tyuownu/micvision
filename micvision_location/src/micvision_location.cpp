#include <micvision_location/micvision_location.h>
// #include <micvision_location/computed_map.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

#include <string.h>

namespace micvision {

inline Eigen::Vector2i floor(const Eigen::Vector3f& v) {
  return Eigen::Vector2i(std::lround(v[0]-0.5), std::lround(v[1]-0.5));
}

LaserScanSample MicvisionLocation::transformPointCloud(
    const Eigen::Quaternionf& transform) {
  PointCloudUV result;
  result.reserve(point_cloud_.size());
  std::vector<int> indices;
  indices.reserve(point_cloud_.size());
  const float resolution = current_map_.getResolution();
  int min_x = width_, max_x = -min_x;
  int min_y = height_, max_y = -min_y;
  for ( const Eigen::Vector3f& point:point_cloud_ ) {
    // result.emplace_back(transform * point);
    const Eigen::Vector2i temp = floor(transform * point / resolution);
    min_x = min_x < temp[0] ? min_x : temp[0];
    min_y = min_y < temp[1] ? min_y : temp[1];
    max_x = max_x > temp[0] ? max_x : temp[0];
    max_y = max_y > temp[1] ? max_y : temp[1];
    result.emplace_back(temp);
    indices.emplace_back(temp[0] + temp[1] * width_);
  }
  return LaserScanSample{result, indices, min_x, max_x, min_y, max_y};
}

MicvisionLocation::MicvisionLocation() {
  ros::NodeHandle nh;

  position_publisher_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1);

  map_sub_  = nh.subscribe("/map", 1, &MicvisionLocation::mapCallback, this);
  scan_sub_ = nh.subscribe("/scan", 1, &MicvisionLocation::scanCallback, this);
  debug_position_sub_ = nh.subscribe(
      "/debug_position", 1, &MicvisionLocation::debugAPosition, this);

  location_server_ =
      nh.advertiseService(LOCATION_SERVICE,
                          &MicvisionLocation::receiveLocation, this);

  std::string service_name;
  nh.param("map_service", service_name, std::string("/static_map"));
  get_map_client_ = nh.serviceClient<nav_msgs::GetMap>(service_name);

  // TODO: using parameter server
  inflation_radius_ = 0.3;
  robot_radius_ = 0.2;
  cost_obstacle_ = 100;

  has_new_map_ = true;
  inflation_markers_ = NULL;
  cached_distances_ = NULL;
  cached_costs_ = NULL;

  // laserscan relative
  handling_lasescan_ = false;
  laserscan_circle_step_ = 6;
  range_step_ = 3;
  laserscan_anglar_step_ = 6.0;   // degree

  min_valid_range_ = 0.0;
  max_valid_range_ = 10.0;
  quick_score_num_ = 8;
  quick_score_ = true;

  dynamic_srv_ = new LocationConfigServer(ros::NodeHandle("~"));
  CallbackType cb = boost::bind(&MicvisionLocation::reconfigureCB, this, _1, _2);
  dynamic_srv_->setCallback(cb);
}

void MicvisionLocation::reconfigureCB(Config &config, uint32_t level) {
  inflation_radius_ = config.inflation_radius;
  robot_radius_ = config.robot_radius;
  laserscan_circle_step_ = config.laserscan_circle_step;
  range_step_ = config.range_step;
  laserscan_anglar_step_ = config.laserscan_anglar_step;
  min_valid_range_ = config.min_valid_range;
  max_valid_range_ = config.max_valid_range;
  quick_score_ = config.quick_score;
  quick_score_num_ = config.quick_score_num;

  return;
}

MicvisionLocation::~MicvisionLocation() {
  delete[] inflation_markers_;
  delete[] cached_distances_;
  delete[] cached_costs_;
}

void MicvisionLocation::mapCallback(const nav_msgs::OccupancyGrid& map) {
  ROS_DEBUG("mapCallback");

  // only inflate the map when the first map or a new map comes
  if ( !getMap() ) {
    ROS_WARN("Could not get a new map, trying to go with the old one...");
    return;
  }

  inflateMap();
}

void MicvisionLocation::scanCallback(const sensor_msgs::LaserScan& scan) {
  // ROS_INFO("scanCallback, scan size: %d", scan.ranges.size());

  if ( ros::ok() && !handling_lasescan_ ) {
    point_cloud_.clear();
    // generate the point_cloud_
    float angle = scan.angle_min;
    for ( int i = 0; i < scan.ranges.size(); i += laserscan_circle_step_ ) {
      const float range = scan.ranges[i];
      if ( min_valid_range_ <= range && range <= max_valid_range_ ) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        point_cloud_.push_back(rotation*( range*Eigen::Vector3f::UnitX() ));
      }

      angle += scan.angle_increment * laserscan_circle_step_;
    }
    // handleLaserScan();

  }
}

void MicvisionLocation::handleLaserScan() {
  handling_lasescan_ = true;
  laserscan_samples_.clear();
  double angle = -M_PI;
  laserscan_samples_.reserve(static_cast<int>(
          PI_2/RADIAN_PRE_DEGREE/laserscan_anglar_step_));
  while ( angle <= M_PI ) {
    // angle = -M_PI + laserscan_anglar_step_ * N
    laserscan_samples_.emplace_back(transformPointCloud(
                Eigen::Quaternionf(
                    Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()))));
    angle += laserscan_anglar_step_ * RADIAN_PRE_DEGREE;
  }
  handling_lasescan_ = false;
}

void MicvisionLocation::scoreLaserScanSamples() {
  // first we need handle the point_cloud_
  handleLaserScan();
  ROS_INFO("start score");
  // ROS_INFO("score the laserscan samples.");
  double score = 0;
  int count = 0;

  // ROS_INFO_STREAM("inflated map data size: " << inflated_map_data_.size());
  const int sample_size = laserscan_samples_[0].point_cloud.size();
  const int step = sample_size / quick_score_num_;
  // for ( int uv = 0; uv < inflated_map_data_.size(); ++uv ) {
  for ( int v = 0; v < height_; v += range_step_ ) {
    for ( int u = 0; u < width_; u += range_step_ ) {
      const int uv = v*width_ + u;
      if ( !inflated_map_data_[uv].first ) {
        continue;
      }

      for ( int i = 0; i < laserscan_samples_.size(); ++i ) {
        const LaserScanSample &sample = laserscan_samples_[i];
        if ( u + sample.min_x <= 1 || u + sample.max_x >= width_ - 1 )
          continue;
        if ( v + sample.min_y <= 1 || v + sample.max_y >= height_ - 1 )
          continue;

        double sample_score = 0;
        if ( quick_score_ ) {
          int object = 0;
          for ( int point_index = 0; point_index < sample_size;
               point_index += step ) {
            if ( inflated_map_data_[sample.indices[point_index]
                + uv].second >= 0.5 )
              ++object;
          }

          if ( object <= quick_score_num_ / 2 )
            continue;
        }
        count++;

        for ( auto point_index : sample.indices )
          sample_score += inflated_map_data_[point_index + uv].second;
        sample_score /= static_cast<double>(sample_size);

        if ( sample_score > score ) {
          score = sample_score;
          best_angle_ = -M_PI + i * laserscan_anglar_step_ * RADIAN_PRE_DEGREE;
          best_position_ = Eigen::Vector2i(u, v);
        }
      }
    }
  }

  geometry_msgs::PoseWithCovarianceStamped init_pose;
  const double x = best_position_[0]*resolution_ + current_map_.getOriginX();
  const double y = best_position_[1]*resolution_ + current_map_.getOriginY();

  init_pose.header.stamp = ros::Time::now();
  init_pose.header.frame_id = "map";

  init_pose.pose.pose.position.x = x;
  init_pose.pose.pose.position.y = y;
  init_pose.pose.pose.position.z = 0;

  init_pose.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(best_angle_);
  init_pose.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.0, 0.0, 0.0, 0.0, 0.06853891945200942 };

  position_publisher_.publish(init_pose);


  ROS_INFO("Best score: %f, angle: %f, Best position: %f, %f, count: %d",
           score, best_angle_/RADIAN_PRE_DEGREE, x, y, count);
}

double MicvisionLocation::scoreASample(const LaserScanSample& sample,
                                       const int u, const int v) {
  double score = 0;
  if ( quick_score_ ) {
    int step = sample.point_cloud.size() / quick_score_num_;
    int object = 0;
    for ( int i = 0; i < sample.point_cloud.size(); i+=step ) {
      if ( current_map_.getRawData(sample.point_cloud[i][0]+v,
                                   sample.point_cloud[i][1]+u) >= 50 )
        object++;
    }
    if ( object <= quick_score_num_/2 ) return 0;
  }
  for ( auto s:sample.point_cloud ) {
    // s = [width, height, z]
    score += current_map_.getRawData(s[0]+v, s[1]+u);
  }
  // ROS_INFO("score: %f", score);
  return score;
}

bool MicvisionLocation::receiveLocation(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  scoreLaserScanSamples();
  res.success = true;
  res.message = "Location success.";
  return true;

  /*
   *char fn[4096];
   *sprintf(fn, "/home/tyu/10101.txt");
   *FILE *fp = fopen(fn, "w");
   *for ( int i = 0; i < current_map_.getHeight(); i++ ) {
   *  for ( int j = 0; j < current_map_.getWidth(); j++ ) {
   *    fprintf(fp, "%d ", current_map_.getData(i, j));
   *  }
   *  fprintf(fp, "\n");
   *}
   *fclose(fp);
   */
}

bool MicvisionLocation::getMap() {
  // new map comming?
  if ( !has_new_map_ )
    return false;

  if ( !get_map_client_.isValid() ) {
    ROS_ERROR("get map client is invalid!");
    return false;
  }

  nav_msgs::GetMap srv;
  if ( !get_map_client_.call(srv) ) {
    ROS_INFO("Could not get a map.");
    return false;
  }
  current_map_.update(srv.response.map);

  ROS_INFO("Now inflation the static map.");
  cell_inflation_radius_ = inflation_radius_ / current_map_.getResolution();
  computeCaches();

  has_new_map_ = false;
  return true;
}

void MicvisionLocation::computeCaches() {
  cached_costs_ = new signed char*[cell_inflation_radius_+2];
  cached_distances_ = new double*[cell_inflation_radius_+2];

  for ( unsigned int i = 0; i < cell_inflation_radius_+2; i++ ) {
    cached_costs_[i] = new signed char[cell_inflation_radius_+2];
    cached_distances_[i] = new double[cell_inflation_radius_+2];
    for ( unsigned int j = 0; j < cell_inflation_radius_+2; j++ ) {
      double d = sqrt(static_cast<double>(i*i+j*j));
      cached_distances_[i][j] = d;
      d /= static_cast<double>(cell_inflation_radius_);
      if ( d>1 ) d = 1;
      cached_costs_[i][j] = (1.0 - d) * cost_obstacle_;
    }
  }
}

void MicvisionLocation::inflateMap() {
  ROS_DEBUG("Infalting the map ...");
  const int map_size = current_map_.getSize();
  width_ = current_map_.getWidth();
  height_ = current_map_.getHeight();
  resolution_ = current_map_.getResolution();

  if ( inflation_markers_ )
    delete[] inflation_markers_;
  inflation_markers_ = new unsigned char[map_size];
  memset(inflation_markers_, 0, map_size*sizeof(unsigned char));

  while ( !inflation_queue_.empty() )
    inflation_queue_.pop();

  for ( int index = 0; index < map_size; index++ ) {
    if ( current_map_.getData(index) > 0 ) {
      unsigned int x, y;
      current_map_.getCoordinates(x, y, index);
      enqueueObstacle(index, x, y);
    }
    /*
     *else if ( current_map_.getData(index) == -1 ) {
     *  inflation_markers_[index] = 1;
     *}
     */
  }

  int count = 0;
  while ( !inflation_queue_.empty() ) {
    const CellData cell = inflation_queue_.top();
    inflation_queue_.pop();

    unsigned int x, y;
    if ( !current_map_.getCoordinates(x, y, cell.index) )
      continue;

    if ( x >= 1 )
      enqueueObstacle(cell.index-1, cell.x, cell.y);

    if ( x < width_ - 1 )
      enqueueObstacle(cell.index+1, cell.x, cell.y);

    if ( y >= 1 )
      enqueueObstacle(cell.index - width_, cell.x, cell.y);

    if ( y < current_map_.getHeight() - 1 )
      enqueueObstacle(cell.index + width_, cell.x, cell.y);
    count++;
  }
  ROS_INFO_STREAM("Inflated " << count << " cells");

  inflated_map_data_.reserve(height_ * width_);
  for ( int u = 0; u < height_; ++u ) {
    for ( int v = 0; v < width_; ++v ) {
      const auto data = current_map_.getData(v, u);
      if ( data >= 0 && data < 50 )
      {
        inflated_map_data_.emplace_back(
            std::make_pair(true,
                           static_cast<double>(data) /
                           static_cast<double>(cost_obstacle_)));
      }
      else
        inflated_map_data_.emplace_back(
            std::make_pair(false,
                           static_cast<double>(data) /
                           static_cast<double>(cost_obstacle_)));
      /*
       *inflated_map_data_.emplace_back(
       *    std::make_pair(data >= 0 && data < 50, data));
       */
    }
  }
}

void MicvisionLocation::enqueueObstacle(const unsigned int index,
                                        const unsigned int x,
                                        const unsigned int y) {
  unsigned int mx, my;
  if ( !current_map_.getCoordinates(mx, my, index) ||
       inflation_markers_[index] != 0 )
    return;

  const double d = distanceLookup(mx, my, x, y);
  if ( d > cell_inflation_radius_ )
    return;

  const CellData cell(d, index, x, y);
  inflation_queue_.push(cell);
  inflation_markers_[index] = 1;
  const signed char value = costLookup(mx, my, x, y);
  current_map_.setData(index, value);
}

inline double MicvisionLocation::distanceLookup(const unsigned int mx,
                                                const unsigned int my,
                                                const unsigned int sx,
                                                const unsigned int sy) const {
  const unsigned int dx = abs(static_cast<int>(mx) - static_cast<int>(sx));
  const unsigned int dy = abs(static_cast<int>(my) - static_cast<int>(sy));

  if ( dx > cell_inflation_radius_ + 1 || dy > cell_inflation_radius_ + 1 ) {
    ROS_ERROR("Error in distanceLookup table! Asked for"
              " (%d, %d), but cell_inflation_radius_ is %d!",
              dx, dy, cell_inflation_radius_);
    return cell_inflation_radius_ + 1;
  }
  return cached_distances_[dx][dy];
}

inline signed char MicvisionLocation::costLookup(const unsigned int mx,
                                                 const unsigned int my,
                                                 const unsigned int sx,
                                                 const unsigned int sy) const {
  const unsigned int dx = abs(static_cast<int>(mx) - static_cast<int>(sx));
  const unsigned int dy = abs(static_cast<int>(my) - static_cast<int>(sy));

  if ( dx > cell_inflation_radius_ + 1 || dy > cell_inflation_radius_ + 1 ) {
    ROS_ERROR("Error in distanceLookup table! Asked for"
              " (%d, %d), but cell_inflation_radius_ is %d!",
              dx, dy, cell_inflation_radius_);
    return 0;
  }
  return cached_costs_[dx][dy];
}

void MicvisionLocation::debugAPosition(const geometry_msgs::Pose2D &pose) {
  handleLaserScan();
  const double origin_x = current_map_.getOriginX();
  const double origin_y = current_map_.getOriginY();
  ROS_INFO("origin: [%f, %f]", origin_x, origin_y);

  const int u = ( pose.x - origin_x ) / resolution_ + 1;
  const int v = ( pose.y - origin_y ) / resolution_ + 1;
  const int angle_index = (pose.theta / 180.0 + 1) *
      M_PI / laserscan_anglar_step_;

  double score;
  const LaserScanSample &sample = laserscan_samples_[angle_index];
  ROS_INFO("angle index: %d, u: %d, v:%d", angle_index, u, v);

  if ( v + sample.min_y <= 1 || v + sample.max_y >= height_ - 1 ||
      u + sample.min_x <= 1 || u + sample.max_x >= width_ - 1 ) {
    ROS_INFO("the laser is out of map's bound.");
    return;
  }

  const int uv = v*width_ + u;

  for ( const auto point_index : sample.indices )
    score += inflated_map_data_[point_index + uv].second;

  score /= static_cast<double>(sample.indices.size());

  ROS_INFO("Position: [%f, %f], angle: %f, score: %f",
           pose.x, pose.y, pose.theta, score);

  ROS_INFO("u: %d, v: %d", u, v );
  for ( auto point_index : sample.indices )
    ROS_INFO("index: %d, score: %d", point_index,
             static_cast<int>( inflated_map_data_[point_index + uv].second ));

  ROS_INFO("\n\n\nnext is the best position.");
  const LaserScanSample &best_sample =
      laserscan_samples_[( best_angle_+M_PI )/laserscan_anglar_step_ + 1];
  const int best_uv = best_position_[1] * width_ + best_position_[0];
  for ( const auto index : best_sample.indices )
    ROS_INFO("index: %d, score: %d", index,
             static_cast<int>( inflated_map_data_[index + best_uv].second ));
}
}  // end namespace micvision
