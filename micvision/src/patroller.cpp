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


#include <micvision/patroller.h>

namespace micvision {
MicvisionPatroller::MicvisionPatroller() {
  ros::NodeHandle n;

  marker_pub_ =
      n.advertise<visualization_msgs::MarkerArray>("landmarks", 10);
  goal_pub_ =
      n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);

  click_sub_ = n.subscribe("/clicked_point", 10,
                           &MicvisionPatroller::clickCallback, this);
  feedback_sub_ = n.subscribe("/move_base/feedback", 10,
                              &MicvisionPatroller::positionFeedback, this);
  extern_goal_sub_ = n.subscribe("/move_base_simple/goal", 1,
                                 &MicvisionPatroller::externGoalCallback, this);

  cmd_start_server_ = n.advertiseService(
      START_PATROLLER, &MicvisionPatroller::receiveStartCommand, this);
  cmd_stop_server_ = n.advertiseService(
      STOP_PATROLLER, &MicvisionPatroller::receiveStopCommand, this);
  cmd_reset_server_ = n.advertiseService(
      RESET_PATROLLER, &MicvisionPatroller::receiveResetCommand, this);
}

MicvisionPatroller::~MicvisionPatroller() {
  landmarks_.clear();
  marker_array_.markers.clear();
}

void MicvisionPatroller::publishMarkerArray() {
  ros::Rate rate(10);

  while ( ros::ok() && true ) {
    marker_pub_.publish(marker_array_);
    rate.sleep();
    ros::spinOnce();
  }
}

void MicvisionPatroller::clickCallback(
    const geometry_msgs::PointStamped::ConstPtr &point) {
  const auto i = landmarks_.size();
  geometry_msgs::Point p;
  p.x = point->point.x;
  p.y = point->point.y;
  ROS_INFO_STREAM("Adding landmark #" << i+1 << " : ("
                  << p.x << ", " << p.y << ");");
  landmarks_.push_back(p);
  visualization_msgs::Marker cylinder;

  {
    cylinder.header.frame_id    = "/map";
    cylinder.header.stamp       = ros::Time::now();
    cylinder.ns                 = "cylinder";
    cylinder.id                 = 2*i;
    cylinder.type               = visualization_msgs::Marker::CYLINDER;
    cylinder.action             = visualization_msgs::Marker::ADD;
    cylinder.pose.orientation.w = 1.0;
    cylinder.scale.x            = 0.5;
    cylinder.scale.y            = 0.5;
    cylinder.scale.z            = 0.01;

    cylinder.color.g            = 1.0f;
    cylinder.color.a            = 0.4f;
    cylinder.lifetime           = ros::Duration();

    cylinder.pose.position.x    = landmarks_[i].x;
    cylinder.pose.position.y    = landmarks_[i].y;
  }
  marker_array_.markers.push_back(cylinder);

  visualization_msgs::Marker text;
  {
    text.header.frame_id    = "/map";
    text.header.stamp       = ros::Time::now();
    text.ns                 = "text";
    text.id                 = 2*i + 1;
    text.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action             = visualization_msgs::Marker::ADD;
    text.pose.orientation.w = 1.0;
    text.scale.x            = 1;
    text.scale.y            = 1;
    text.scale.z            = 1;

    text.color.r            = 1.0f;
    text.color.a            = 1.0f;
    text.lifetime           = ros::Duration();

    text.pose.position.x    = landmarks_[i].x + 0.2;
    text.pose.position.y    = landmarks_[i].y + 0.2;

    text.text               = boost::lexical_cast<std::string>(i+1);
  }
  marker_array_.markers.push_back(text);

  // marker_pub_.publish(marker_array_);
}

void MicvisionPatroller::publishGoal(const bool first_input) {
  if ( !first_input )
    current_index_++;

  if ( current_index_ >= landmarks_.size() )
    current_index_ = 0;

  geometry_msgs::PoseStamped goal;
  {
    goal.header.stamp       = ros::Time::now();
    goal.header.frame_id    = "map";
    goal.pose.position.x    = landmarks_[current_index_].x;
    goal.pose.position.y    = landmarks_[current_index_].y;
    goal.pose.orientation.w = 1.0f;
  }
  move_base_msgs::MoveBaseActionGoal move_base_action_goal;
  {
    move_base_action_goal.header.stamp = ros::Time::now();
    move_base_action_goal.goal_id.stamp.sec = 0;
    move_base_action_goal.goal_id.stamp.nsec = 0;
    move_base_action_goal.goal_id.id = "";
    move_base_action_goal.goal.target_pose = goal;
  }

  goal_pub_.publish(move_base_action_goal);
}


void MicvisionPatroller::positionFeedback(
    const move_base_msgs::MoveBaseActionFeedback::ConstPtr &position ) {
  if ( !patrolling_ ) return;
  if ( distance(position->feedback.base_position.pose.position,
                landmarks_[current_index_]) < 0.3 ) {
    publishGoal(false);
  }
}

bool MicvisionPatroller::receiveStartCommand(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  if ( landmarks_.size() < 2 ) {
    ROS_ERROR("The landmarks number is less than 2,"
              " please entry more landmarks!");
    res.success = false;
    res.message = "There are at least have 2 landmarks.";
    return false;
  } else {
    patrolling_ = true;
    res.success = true;
    res.message = "Start patrol.";
    publishGoal(true);
    return true;
  }
}

bool MicvisionPatroller::receiveStopCommand(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  patrolling_ = false;
  res.success = true;
  res.message = "Stop patrol.";
  return true;
}

bool MicvisionPatroller::receiveResetCommand(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  patrolling_ = false;
  landmarks_.clear();

  marker_array_.markers.clear();
  current_index_ = 0;

  res.success = true;
  res.message = "Reset patrol.";
  return true;
}
void MicvisionPatroller::externGoalCallback(
    const geometry_msgs::PoseStamped &goal) {
  patrolling_ = false;
}
}  // end namespace micvision

int main(int argc, char **argv)
{
  ros::init(argc, argv, "micvision_patroller");
  ros::NodeHandle n;

  micvision::MicvisionPatroller micvision_patroller;

  micvision_patroller.publishMarkerArray();

  ros::spin();
  return 0;
}
