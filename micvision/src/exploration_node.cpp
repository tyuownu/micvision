#include <ros/ros.h>

#include <micvision/exploration.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "micvision_exploration_node");
  ros::NodeHandle n;

  micvision::MicvisionExploration exploration;

  ros::spin();
  return 0;
}
