#ifndef MICVISION_PATROLLER_H_
#define MICVISION_PATROLLER_H_
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <micvision/commands.h>
#include <vector>

namespace micvision {

class MicvisionPatroller {
 public:
  MicvisionPatroller();
  ~MicvisionPatroller();

 private:
  void clickCallback(const geometry_msgs::PointStamped::ConstPtr &point);
  void publishGoal(const bool first_input);
  void positionFeedback(
      const move_base_msgs::MoveBaseActionFeedback::ConstPtr &position);

  bool receiveStartCommand(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res);

  bool receiveStopCommand(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res);

  bool receiveResetCommand(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res);


 private:
  std::vector<geometry_msgs::Point>    landmarks_;
  visualization_msgs::MarkerArray      marker_array_;

  ros::Publisher                       marker_pub_;
  ros::Publisher                       goal_pub_;
  ros::Subscriber                      click_sub_;
  ros::Subscriber                      feedback_sub_;
  ros::ServiceServer                   cmd_start_server_;
  ros::ServiceServer                   cmd_stop_server_;
  ros::ServiceServer                   cmd_reset_server_;

  int                                  current_index_ = 0;
  bool                                 patrolling_ = false;
};

double distance(const geometry_msgs::Point &p1,
                const geometry_msgs::Point &p2) {
  return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}
}  // namespace micvision

#endif  // MICVISION_PATROLLER_H_
