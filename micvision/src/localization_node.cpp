#include <ros/ros.h>
#include <micvision/localization.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "micvision_localization_node");
  ros::NodeHandle n;
  micvision::MicvisionLocalization mic_localization;

  mic_localization.tracking();
  ros::spin();
  return 0;
}
