#include <ros/ros.h>
#include <micvision_location/micvision_location.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "micvision_location_node");
  micvision::MicvisionLocation mic_location;

  ros::spin();
  return 0;
}
