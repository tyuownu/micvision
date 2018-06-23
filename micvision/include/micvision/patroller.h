/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2018, Micvision, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/



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
  void publishMarkerArray();

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

  void externGoalCallback(const geometry_msgs::PoseStamped &goal);



 private:
  std::vector<geometry_msgs::Point>    landmarks_;
  visualization_msgs::MarkerArray      marker_array_;

  ros::Publisher                       marker_pub_;
  ros::Publisher                       goal_pub_;
  ros::Subscriber                      click_sub_;
  ros::Subscriber                      feedback_sub_;
  ros::Subscriber                      extern_goal_sub_;
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
