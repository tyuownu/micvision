#ifndef MICVISION_LOCATION_H_
#define MICVISION_LOCATION_H_
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <micvision_location/LocationAction.h>
#include <micvision_location/commands.h>

typedef actionlib::SimpleActionServer<micvision_location::LocationAction> LocationActionServer;

namespace micvision {
class MicvisionLocation {
 public:
    MicvisionLocation();
    ~MicvisionLocation() {}

    void mapCallback(const nav_msgs::OccupancyGrid& map);
    void scanCallback(const sensor_msgs::LaserScan& scan);
    void receiveLocationGoal(const micvision_location::LocationGoal::ConstPtr&);

 private:
    ros::Publisher position_publisher_;
    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;
    LocationActionServer* location_action_server_;
};
}  // namespace micvision
#endif  // end MICVISION_LOCATION_H_
