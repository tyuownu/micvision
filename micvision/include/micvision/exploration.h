#ifndef MICVISION_EXPLORATION_H_
#define MICVISION_EXPLORATION_H_
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_loader.h>

#include <micvision/ExplorationAction.h>
#include <micvision/grid_map.h>
#include <micvision/commands.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <Eigen/Core>

namespace micvision {

using Action = micvision::ExplorationAction;
using Server = actionlib::SimpleActionServer<Action>;
using Client = actionlib::SimpleActionClient<Action>;

class MicvisionExploration
{
 public:
  MicvisionExploration();
  ~MicvisionExploration();

 private:
  bool receiveStop(std_srvs::Trigger::Request &req,
                   std_srvs::Trigger::Response &res);
  bool receiveStopExploration(std_srvs::Trigger::Request &req,
                              std_srvs::Trigger::Response &res);
  bool receivePause(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res);
  void receiveExplorationGoal(
      const micvision::ExplorationGoal::ConstPtr &goal);
  void mapCallback(const nav_msgs::OccupancyGrid& global_map);
  void scanCallback(const sensor_msgs::LaserScan& scan);
  void externGoalCallback(const geometry_msgs::PoseStamped&);
  int scoreLine(double, double);
  Pixel world2pixel(const Point& point) const;
  Point pixel2world(const Pixel& pixel) const;

  bool setCurrentPosition();
  void stop();
  bool preparePlan();

  // start pixel is [0, 0]
  std::vector<Pixel> bresenham(const Pixel& end);

  bool findSector();
  void searchDeeper();
  void updateGoalCoordinates(const unsigned int&);

 private:
  // Everything related to ROS
  tf::TransformListener tf_listener_;
  ros::ServiceServer stop_server_;
  ros::ServiceServer pause_server_;
  ros::ServiceServer stop_exploration_server_;

  std::string map_frame_;
  std::string robot_frame_;
  std::string explore_action_topic_;

  Server* exploration_action_server_;

  // Current status and goals
  bool receive_new_map_;
  bool is_paused_;
  bool is_stopped_;
  unsigned int goal_index_;
  unsigned int start_index_;
  double update_frequency_;

  double angles_;

  Point goal_point_;
  double robot_theta_;
  unsigned int count_;
  unsigned int interval_;
  sensor_msgs::LaserScan scan_;
  bool exploration_running_;

  // Everything related to the global map and plan
  GridMap current_map_;

  ros::Publisher goal_publisher_;
  ros::Publisher stop_publisher_;
  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber extern_goal_sub_;

  Pixel robot_pixel_;
  Point robot_point_;
};
}  // end namespace micvision
#endif  // end MICVISION_EXPLORATION_H_
