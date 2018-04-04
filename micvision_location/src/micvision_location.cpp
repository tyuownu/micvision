#include <micvision_location/micvision_location.h>
#include <micvision_location/computed_map.h>
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

  location_action_server_ = new
      LocationActionServer(LOCATION_ACTION,
                           boost::bind(&MicvisionLocation::receiveLocationGoal,
                                       this, _1), false);
  location_action_server_->start();

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
  update_laserscan_ = false;
  laserscan_circle_step_ = 6;
  range_step_ = 3;
  laserscan_anglar_step_ = 6.0*M_PI/180;  // double

  min_valid_range_ = 0.0;
  max_valid_range_ = 10.0;
}

MicvisionLocation::~MicvisionLocation() {
  delete location_action_server_;
  delete[] inflation_markers_;
  delete[] cached_distances_;
  delete[] cached_costs_;
}

void MicvisionLocation::mapCallback(const nav_msgs::OccupancyGrid& map) {
  ROS_DEBUG("mapCallback");
}

void MicvisionLocation::scanCallback(const sensor_msgs::LaserScan& scan) {
  // ROS_INFO("scanCallback, scan size: %d", scan.ranges.size());

  if ( ros::ok() && update_laserscan_ ) {
    point_cloud_.clear();
    update_laserscan_ = false;
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

    handleLaserScan();

  }
}

void MicvisionLocation::handleLaserScan() {
  laserscan_samples_.clear();
  double angle = -M_PI;
  laserscan_samples_.reserve(static_cast<int>(PI_2/laserscan_anglar_step_));
  while ( angle < M_PI ) {
    // angle = -M_PI + laserscan_anglar_step_ * N
    laserscan_samples_.emplace_back(transformPointCloud(
                Eigen::Quaternionf(
                    Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()))));
    angle += laserscan_anglar_step_;
  }
  ROS_INFO("start score");
  scoreLaserScanSamples();
}

void MicvisionLocation::scoreLaserScanSamples() {
  // ROS_INFO("score the laserscan samples.");
  double score = 0;
  int count = 0;

  ROS_INFO_STREAM("inflated map data size: " << inflated_map_data_.size());
  const int N = 8, half_N = N/2;
  const int sample_size = laserscan_samples_[0].point_cloud.size();
  const int step = sample_size / N;
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
        int object = 0;
        for ( int point_index = 0; point_index < sample_size;
             point_index += step ) {
          if ( inflated_map_data_[sample.indices[point_index]
                                  + uv].second >= 50 )
            ++object;
        }

        if ( object <= half_N )
          continue;
        count++;

        for ( auto point_index : sample.indices )
          sample_score += inflated_map_data_[point_index + uv].second;
        sample_score /= static_cast<double>(sample_size) * 100.0;

        if ( sample_score > score ) {
          score = sample_score;
          best_angle_ = -M_PI + i * laserscan_anglar_step_;
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
           score, best_angle_*180/M_PI, x, y, count);
}

double MicvisionLocation::scoreASample(const LaserScanSample& sample,
                                       const int u, const int v) {
  double score = 0;
  /*
   *  int N = 20;
   *  int step = sample.point_cloud.size()/N;
   *
   *  int no_object = 0;
   *  for ( int i = 0; i < sample.point_cloud.size(); i+=step) {
   *    auto t = current_map_.getRawData(sample.point_cloud[i][0]+v,
   *                                     sample.point_cloud[i][1]+u);
   *    if ( t > 50 )
   *      score += t;
   *    else
   *      no_object++;
   *
   *    if ( no_object >= N/3 )
   *      return 0;
   *  }
   */
  int N = 8;
  int step = sample.point_cloud.size() / N;
  int object = 0;
  for ( int i = 0; i < sample.point_cloud.size(); i+=step ) {
    if ( current_map_.getRawData(sample.point_cloud[i][0]+v,
                                 sample.point_cloud[i][1]+u) >= 50 )
      object++;
  }
  if ( object <= N/2 ) return 0;
  for ( auto s:sample.point_cloud ) {
    // s = [width, height, z]
    score += current_map_.getRawData(s[0]+v, s[1]+u);
  }
  // ROS_INFO("score: %f", score);
  return score;
}

void MicvisionLocation::receiveLocationGoal(
    const micvision_location::LocationGoal::ConstPtr &goal) {
  static bool first_location_goal = true;

  if ( !getMap() && !first_location_goal ) {
    update_laserscan_ = true;
    ROS_WARN("Could not get a new map, trying to go with the old one...");
    return;
  }

  inflateMap();

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
  update_laserscan_ = true;
  first_location_goal = false;
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
    CellData cell = inflation_queue_.top();
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
      auto data = current_map_.getData(v, u);
      if ( data >= 0 && data < 50 )
      {
        inflated_map_data_.emplace_back(std::make_pair(true, data));
      }
      else
        inflated_map_data_.emplace_back(std::make_pair(false, data));
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

  CellData cell(d, index, x, y);
  inflation_queue_.push(cell);
  inflation_markers_[index] = 1;
  const signed char value = costLookup(mx, my, x, y);
  current_map_.setData(index, value);
}

inline double MicvisionLocation::distanceLookup(const unsigned int mx,
                                                const unsigned int my,
                                                const unsigned int sx,
                                                const unsigned int sy) {
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
                                                 const unsigned int sy) {
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
}  // end namespace micvision
