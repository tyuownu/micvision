#include <micvision_location/micvision_location.h>
#include <micvision_location/computed_map.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <string.h>

namespace micvision {

inline Eigen::Vector2i floor(const Eigen::Vector3f& v) {
  return Eigen::Vector2i(std::lround(v[0]-0.5), std::lround(v[1]-0.5));
}

LaserScanSample MicvisionLocation::transformPointCloud(
    const Eigen::Quaternionf& transform) {
  PointCloudUV result;
  result.reserve(point_cloud_.size());
  const float resolution = current_map_.getResolution();
  int min_x = current_map_.getWidth(), max_x = -min_x;
  int min_y = current_map_.getHeight(), max_y = -min_y;
  for ( const Eigen::Vector3f& point:point_cloud_ ) {
    // result.emplace_back(transform * point);
    const Eigen::Vector2i temp = floor(transform * point / resolution);
    min_x = min_x < temp[0] ? min_x : temp[0];
    min_y = min_y < temp[1] ? min_y : temp[1];
    max_x = max_x > temp[0] ? max_x : temp[0];
    max_y = max_y > temp[1] ? max_y : temp[1];
    result.emplace_back(temp);
  }
  return LaserScanSample{result, min_x, max_x, min_y, max_y};
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
        // ROS_INFO_STREAM("range: " << range << ", points: " << (rotation*(range*Eigen::Vector3f::UnitX())).transpose());
      }
      // ROS_INFO_STREAM("ange: " << range);

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
    laserscan_samples_.push_back(std::make_pair(angle, transformPointCloud(
                Eigen::Quaternionf(
                    Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ())))));
    angle += laserscan_anglar_step_;
  }
  ROS_INFO("start score");
  scoreLaserScanSamples();
}

void MicvisionLocation::scoreLaserScanSamples() {
  // ROS_INFO("score the laserscan samples.");
  double score = 0;
  int count = 0;
  for ( int u = 1; u < current_map_.getHeight(); u += range_step_ ) {
    for ( int v = 1; v < current_map_.getWidth(); v += range_step_ ) {
  /*
   *for ( int u = 199; u < 203; u += range_step_ ) {
   *  for ( int v = 199; v < 203; v += range_step_ ) {
   */
      if ( current_map_.getData(v, u) >= 50 || current_map_.getData(v, u) == -1 )
        continue;
      for ( auto sample : laserscan_samples_ ) {
        auto temp = sample.second;
        // ROS_INFO("sample: %d, %d, %d, %d", temp.min_x, temp.min_y, temp.max_x, temp.max_y);
        if ( u + temp.min_y <= 1 ||
            u + temp.max_y >= current_map_.getHeight() )
          continue;
        if ( v + temp.min_x <= 1 ||
            v + temp.max_x >= current_map_.getWidth() )
          continue;
        count++;
        double temp_score = scoreASample(temp, u, v);
        // ROS_INFO("angle: %f, position: (%d, %d), score: %f", sample.first, u, v, temp_score);
        if ( temp_score > score ) {
          score = temp_score;
          best_angle_ = sample.first;
          best_position_ = Eigen::Vector2i(u, v);
        }
      }
    }
  }

  ROS_INFO("Best score: %f, angle: %f, Best position: %d, %d, count: %d", score, best_angle_, best_position_[1], best_position_[0], count);
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
    // ROS_INFO("(%d, %d)+(%d, %d)value: %d",s[0],s[1],v,u, current_map_.getData(s[0]+v, s[1]+u));
    // s = [width, height, z]
    score += current_map_.getRawData(s[0]+v, s[1]+u);
  }
  // ROS_INFO("score: %f", score);
  return score;
}

void MicvisionLocation::receiveLocationGoal(
    const micvision_location::LocationGoal::ConstPtr &goal) {

  update_laserscan_ = true;
  if ( !getMap() ) {
    ROS_WARN("Could not get a new map, trying to go with the old one...");
    return;
  }

  inflateMap();
  ComputedMapStack stack(current_map_);

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

    if ( x < current_map_.getWidth() - 1 )
      enqueueObstacle(cell.index+1, cell.x, cell.y);

    if ( y >= 1 )
      enqueueObstacle(cell.index-current_map_.getWidth(), cell.x, cell.y);

    if ( y < current_map_.getHeight() - 1 )
      enqueueObstacle(cell.index+current_map_.getWidth(), cell.x, cell.y);
    count++;
  }
  ROS_INFO_STREAM("Inflated " << count << " cells");
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
