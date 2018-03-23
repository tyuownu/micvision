#include <micvision_location/micvision_location.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
}

void MicvisionLocation::mapCallback(const nav_msgs::OccupancyGrid& map) {
  ROS_DEBUG("mapCallback");
}

void MicvisionLocation::scanCallback(const sensor_msgs::LaserScan& scan) {
  ROS_DEBUG("scanCallback");
}

void MicvisionLocation::receiveLocationGoal(
    const micvision_location::LocationGoal::ConstPtr &goal) {
  ROS_DEBUG("receiveLocationGoal");
}
}  // end namespace micvision
