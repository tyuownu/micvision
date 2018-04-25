#include <micvision/location.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

#include <string.h>
#include <mutex>

namespace micvision {

static std::mutex mtx;

std::vector<Pixel> MicvisionLocation::bresenham(const Pixel &start,
                                                const Pixel &end) {
  int x = start(0), y = start(1);
  int dx = abs(end(0) - x);
  int dy = abs(end(1) - y);

  const int ux = end(0) > x ? 1 : -1;
  const int uy = end(1) > y ? 1 : -1;

  bool exchange = false;      // exchange dx and dy?
  if ( dy > dx ) {
    std::swap(dx, dy);
    exchange = true;
  }

  int p = 2 * dy - dx;
  std::vector<Pixel> result;

  for ( int i = 0; i <= dx; ++i ) {
    // ROS_INFO("x: %d, y: %d", x, y);
    if ( x <= 5 || x >= current_map_.getWidth() - 5 ||
        y <= 5 || y >= current_map_.getHeight() - 5 )
      break;
    result.push_back(Pixel(x, y));
    if ( p >= 0 ) {
      if ( !exchange )
        y += uy;
      else
        x += ux;
      p -= 2*dx;
    }

    if ( !exchange )
      x += ux;
    else
      y += uy;
    p += 2*dy;
  }
  return result;
}

inline Pixel floor(const Eigen::Vector3f& v) {
  return Pixel(std::lround(v[0]-0.5), std::lround(v[1]-0.5));
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
  for ( const Eigen::Vector3f& point : point_cloud_ ) {
    // result.emplace_back(transform * point);
    const Pixel temp = floor(transform * point / resolution);
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

  map_sub_  = nh.subscribe("/map", 5, &MicvisionLocation::mapCallback, this);
  scan_sub_ = nh.subscribe("/scan", 5, &MicvisionLocation::scanCallback, this);
  odom_sub_ = nh.subscribe("/odom", 10, &MicvisionLocation::odomCallback, this);
  debug_position_sub_ = nh.subscribe(
      "/debug_position", 1, &MicvisionLocation::debugAPosition, this);

  location_server_ =
      nh.advertiseService(LOCATION_SERVICE,
                          &MicvisionLocation::receiveLocation, this);

  std::string service_name;
  nh.param("map_service", service_name, std::string("/static_map"));
  get_map_client_ = nh.serviceClient<nav_msgs::GetMap>(service_name);

  ros::NodeHandle nh_pravite("~/");
  nh_pravite.param("map_frame", map_frame_, std::string("map"));
  nh_pravite.param("robot_frame", robot_frame_, std::string("base_link"));
  nh_pravite.param("tracking_frequency", tracking_frequency_, 1.0);

  map_frame_ = tf_listener_.resolve(map_frame_);
  robot_frame_ = tf_listener_.resolve(robot_frame_);

  dynamic_srv_ = new LocationConfigServer(ros::NodeHandle("~"));
  CallbackType cb = boost::bind(&MicvisionLocation::reconfigureCB,
                                this, _1, _2);
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

void MicvisionLocation::odomCallback(const nav_msgs::Odometry &odom) {
  big_angle_twist_ =
      (std::abs(odom.twist.twist.angular.z) >= 0.1) ? true : false;
}

void MicvisionLocation::scanCallback(const sensor_msgs::LaserScan& scan) {
  // ROS_INFO("scanCallback, scan size: %d", scan.ranges.size());

  mtx.lock();
  if ( ros::ok() ) {
    point_cloud_.clear();
    // generate the point_cloud_
    float angle = scan.angle_min;
    for ( int i = 0; i < scan.ranges.size(); i += laserscan_circle_step_ ) {
      const float range = scan.ranges[i];
      if ( min_valid_range_ <= range && range <= max_valid_range_ ) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        point_cloud_.push_back(rotation*(range*Eigen::Vector3f::UnitX()));
      }

      angle += scan.angle_increment * laserscan_circle_step_;
    }
    // handleLaserScan();
  }
  mtx.unlock();
}

void MicvisionLocation::handleLaserScan() {
  mtx.lock();
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
  mtx.unlock();
}

void MicvisionLocation::scoreLaserScanSamples() {
  // first we need handle the point_cloud_
  handleLaserScan();
  ROS_INFO("start score");
  // ROS_INFO("score the laserscan samples.");
  double score = 0.5;
  int count = 0;

  // ROS_INFO_STREAM("inflated map data size: " << inflated_map_data_.size());
  const int sample_size = laserscan_samples_[0].point_cloud.size();
  const int step = sample_size / quick_score_num_;
  // for ( int uv = 0; uv < inflated_map_data_.size(); ++uv ) 
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
                + uv].second >= 0.5 ) {
              // if ( validPosition(uv, sample.indices[point_index]) )
                ++object;
            }
          }

          if ( object <= quick_score_num_ / 2 )
            continue;
        }
        count++;

        for ( auto point_index : sample.indices )
          sample_score += inflated_map_data_[point_index + uv].second;
        sample_score /= static_cast<double>(sample_size);

        if ( sample_score > score ) {
          if ( quick_score_ ) {
            int object = 0;
            for ( int point_index = 0; point_index < sample_size;
                 point_index += step) {
              if ( validPosition(uv, sample.indices[point_index]) )
                ++object;
            }
            if ( object <= quick_score_num_ / 2 )
              continue;
          }

          score = sample_score;
          best_angle_ = -M_PI + i * laserscan_anglar_step_ * RADIAN_PRE_DEGREE;
          best_position_ = Pixel(u, v);
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

bool MicvisionLocation::validPosition(const int uv, const int index) {
  Pixel start(uv % width_, uv / width_),
        end((uv + index) % width_, (uv + index) / width_);
  /*
   *ROS_INFO("start: %d, %d", start(0), start(1));
   *ROS_INFO("end: %d, %d", end(0), end(1));
   */

  const auto line = bresenham(start, end);

  int i;
  for ( i = 0; i < line.size(); ++i ) {
    const auto l = line[i](0) + line[i](1) * width_;
    if ( inflated_map_data_[l].second > 0.5 )
      break;
  }

  if ( line.size() - i > 5 )
    return false;
  else
    return true;
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
  // handleLaserScan();
  mtx.lock();
  LaserScanSample sample = transformPointCloud(Eigen::Quaternionf(
          Eigen::AngleAxisf(pose.theta, Eigen::Vector3f::UnitZ())));
  mtx.unlock();
  const double origin_x = current_map_.getOriginX();
  const double origin_y = current_map_.getOriginY();
  ROS_DEBUG("origin: [%f, %f]", origin_x, origin_y);

  const int u = ( pose.x - origin_x ) / resolution_ + 1;
  const int v = ( pose.y - origin_y ) / resolution_ + 1;

  double score = 0;
  ROS_DEBUG("u: %d, v:%d", u, v);

  /*
   *if ( v + sample.min_y <= 1 || v + sample.max_y >= height_ - 1 ||
   *    u + sample.min_x <= 1 || u + sample.max_x >= width_ - 1 ) {
   *  ROS_DEBUG("the laser is out of map's bound.");
   *  return 0.0f;
   *}
   */

  const int uv = v*width_ + u;

  for ( const auto point_index : sample.indices ) {
    const auto index = point_index + uv;
    if ( index >= 0 && index < inflated_map_data_.size() )
      score += inflated_map_data_[point_index + uv].second;
  }

  score /= static_cast<double>(sample.indices.size());

  ROS_DEBUG("Position: [%f, %f], angle: %f, score: %f",
            pose.x, pose.y, pose.theta, score);
  current_position_score_ = score;

/*
 *  ROS_INFO("u: %d, v: %d", u, v );
 *  for ( auto point_index : sample.indices )
 *    ROS_INFO("index: %d, score: %d", point_index,
 *             static_cast<int>( inflated_map_data_[point_index + uv].second ));
 *
 *  ROS_INFO("\n\n\nnext is the best position.");
 *  const LaserScanSample &best_sample =
 *      laserscan_samples_[( best_angle_+M_PI )/laserscan_anglar_step_ + 1];
 *  const int best_uv = best_position_[1] * width_ + best_position_[0];
 *  for ( const auto index : best_sample.indices )
 *    ROS_INFO("index: %d, score: %d", index,
 *             static_cast<int>( inflated_map_data_[index + best_uv].second ));
 */
}

void MicvisionLocation::tracking() {
  ros::Rate rate(tracking_frequency_);
  tf::StampedTransform transform;
  int num = 0;
  while ( true && ros::ok() ) {
    // TODO: first to get the current location
    if ( big_angle_twist_ ) {
      ROS_DEBUG("anglar twist big...");
      num = 0;
    } else {
      try {
        ros::Time now = ros::Time::now();
        tf_listener_.waitForTransform(map_frame_, robot_frame_,
                                      now, ros::Duration(3.0));
        tf_listener_.lookupTransform(map_frame_, robot_frame_,
                                     now, transform);
        /*
         *tf_listener_.lookupTransform(map_frame_, robot_frame_,
         *                             ros::Time(0), transform);
         */
      } catch ( tf::TransformException ex ) {
        ROS_ERROR("Could not get robot position: %s", ex.what());
      }
      geometry_msgs::Pose2D pose;
      pose.x = transform.getOrigin().x();
      pose.y = transform.getOrigin().y();
      pose.theta = getYaw(transform.getRotation());

      debugAPosition(pose);
      ROS_DEBUG("current position score: %f", current_position_score_);
      if ( current_position_score_ < 0.5 )
        ++num;
      else
        num = 0;

      if ( num > 10 ) {
        scoreLaserScanSamples();
        num = 0;
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
  // score current location
}
}  // end namespace micvision
