#include <ros/ros.h>
#include <micvision/location.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "micvision_location_node");
  ros::NodeHandle n;
  micvision::MicvisionLocation mic_location;

  mic_location.tracking();
  ros::spin();
  return 0;
}
