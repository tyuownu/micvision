// ros
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>

// micvision
#include <micvision/ExplorationAction.h>
#include <micvision/commands.h>
#include <micvision/exploration.h>


micvision::Client* explore_client;

bool receiveCommand(std_srvs::Trigger::Request& req,
                    std_srvs::Trigger::Response& res) {
  /*
  geometry_msgs::Twist cmd_vel;
  ros::NodeHandle n;
  ros::Publisher vel_pub;
  ros::Rate r(10);
  int t = 0;

  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

  while ( t++ < 32 && ros::ok() ) {
    cmd_vel.linear.x = 0.0f;
    cmd_vel.linear.y = 0.0f;
    cmd_vel.angular.z = 0.5f;

    vel_pub.publish(cmd_vel);
    r.sleep();
  }
  */

  micvision::ExplorationGoal goal;
  explore_client->sendGoal(goal);
  res.success = true;
  res.message = "Send ExploreGoal to Navigator.";
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "micvision_exploration_client");
  ros::NodeHandle n;

  ros::ServiceServer cmd_server =
    n.advertiseService(START_EXPLORATION_SERVICE, &receiveCommand);
  explore_client = new micvision::Client(EXPLORATION_ACTION, true);
  explore_client->waitForServer();

  ros::spin();

  delete explore_client;
  return 0;
}
