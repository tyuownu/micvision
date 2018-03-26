#include <micvision_location/micvision_location.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <string.h>

namespace micvision {
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

  has_new_map_ = false;
  inflation_markers_ = NULL;
  cached_distances_ = NULL;
  cached_costs_ = NULL;
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
  ROS_DEBUG("scanCallback");
}

void MicvisionLocation::receiveLocationGoal(
    const micvision_location::LocationGoal::ConstPtr &goal) {

  if ( !getMap() )
    ROS_WARN("Could not get a new map, trying to go with the old one...");

  inflateMap();

}

bool MicvisionLocation::getMap() {
  if ( has_new_map_ )
    return true;

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

  has_new_map_ = true;
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
      printf("%f " , d);
      d /= static_cast<double>(cell_inflation_radius_);
      if ( d>1 ) d = 1;
      cached_costs_[i][j] = (1.0 - d) * cost_obstacle_;
    }
  }
}

void MicvisionLocation::inflateMap() {
  ROS_DEBUG("Infalting the map ...");
  int map_size = current_map_.getSize();

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
    } else if ( current_map_.getData(index) == -1 ) {
      inflation_markers_[index] = 1;
    }
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

void MicvisionLocation::enqueueObstacle(unsigned int index,
                                        unsigned int x,
                                        unsigned int y) {
  unsigned int mx, my;
  if ( !current_map_.getCoordinates(mx, my, index) ||
       inflation_markers_[index] != 0 )
    return;

  double d = distanceLookup(mx, my, x, y);
  if ( d > cell_inflation_radius_ )
    return;

  CellData cell(d, index, x, y);
  inflation_queue_.push(cell);
  inflation_markers_[index] = 1;
  signed char value = costLookup(mx, my, x, y);
  current_map_.setData(index, value);
}

inline double MicvisionLocation::distanceLookup(unsigned int mx,
                                                unsigned int my,
                                                unsigned int sx,
                                                unsigned int sy) {
  unsigned int dx = abs(static_cast<int>(mx) - static_cast<int>(sx));
  unsigned int dy = abs(static_cast<int>(my) - static_cast<int>(sy));

  if ( dx > cell_inflation_radius_ + 1 || dy > cell_inflation_radius_ + 1 ) {
    ROS_ERROR("Error in distanceLookup table! Asked for"
              " (%d, %d), but cell_inflation_radius_ is %d!",
              dx, dy, cell_inflation_radius_);
    return cell_inflation_radius_ + 1;
  }
  return cached_distances_[dx][dy];
}

inline signed char MicvisionLocation::costLookup(unsigned int mx,
                                                 unsigned int my,
                                                 unsigned int sx,
                                                 unsigned int sy) {
  unsigned int dx = abs(static_cast<int>(mx) - static_cast<int>(sx));
  unsigned int dy = abs(static_cast<int>(my) - static_cast<int>(sy));

  if ( dx > cell_inflation_radius_ + 1 || dy > cell_inflation_radius_ + 1 ) {
    ROS_ERROR("Error in distanceLookup table! Asked for"
              " (%d, %d), but cell_inflation_radius_ is %d!",
              dx, dy, cell_inflation_radius_);
    return 0;
  }
  return cached_costs_[dx][dy];
}
}  // end namespace micvision
