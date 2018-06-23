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

bool receiveCommand(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
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

int main(int argc, char **argv)
{
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
