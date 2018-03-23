#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <micvision_location/LocationAction.h>
#include <std_srvs/Trigger.h>

#include <micvision_location/commands.h>

typedef actionlib::SimpleActionClient<micvision_location::LocationAction> LocationClient;

LocationClient *location_client;

bool receiveCommand(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  micvision_location::LocationGoal goal;
  location_client->sendGoal(goal);
  res.success = true;
  res.message = "Start location...";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Location");
  ros::NodeHandle n;

  ros::ServiceServer cmd_server =
    n.advertiseService(LOCATION_SERVICE, &receiveCommand);
  location_client = new LocationClient(LOCATION_ACTION, true);
  location_client->waitForServer();

  ros::spin();

  delete location_client;
  return 0;
}
