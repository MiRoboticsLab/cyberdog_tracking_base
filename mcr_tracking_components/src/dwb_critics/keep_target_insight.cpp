/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "mcr_tracking_components/dwb_critics/keep_target_insight.hpp"

#include <string>
#include <vector>

#include "angles/angles.h"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/robot_utils.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mcr_tracking_components::KeepTargetInsightCritic,
                       dwb_core::TrajectoryCritic)

namespace mcr_tracking_components {

inline double hypot_sq(double dx, double dy) { return dx * dx + dy * dy; }

void KeepTargetInsightCritic::onInit() {
  normal_sacle_ = 0.0;
  inuse_ = true;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  lookahead_time_ = nav_2d_utils::searchAndGetParam(
      node, dwb_plugin_name_ + "." + name_ + ".lookahead_time", -1.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/chargetolidar_transformed", rclcpp::SensorDataQoS(),
      std::bind(&KeepTargetInsightCritic::poseCallback, this,
                std::placeholders::_1));
  RCLCPP_INFO(
      node->get_logger(),
      "Keep target insight critic subscribed to tracking pose: tracking_pose");
  service_ = node->create_service<std_srvs::srv::SetBool>(
      "is_target_insight_inused",
      std::bind(&KeepTargetInsightCritic::useCriticCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  reset();
  normal_sacle_ = getScale();
}

void KeepTargetInsightCritic::reset() { valid_data_ = false; }

bool KeepTargetInsightCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & /*vel*/,
  const geometry_msgs::msg::Pose2D & /*goal*/,
  const nav_2d_msgs::msg::Path2D &)
{
  if(inuse_){
    goal_yaw_ = atan2(latest_pose_.pose.position.y, latest_pose_.pose.position.x);
  }else{
    goal_yaw_ = tf2::getYaw(latest_pose_.pose.orientation);
  }
  cur_yaw_ = pose.theta;
  return true;
}

double KeepTargetInsightCritic::scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D &traj) {
  // If we're not sufficiently close to the goal, we don't care what the twist
  // is
  if (!valid_data_) {
    return 0.0;
  }
  return scoreRotation(traj);
}

double KeepTargetInsightCritic::scoreRotation(
    const dwb_msgs::msg::Trajectory2D &traj) {
  if (traj.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory.");
  }

  double end_yaw;
  if (lookahead_time_ >= 0.0) {
    geometry_msgs::msg::Pose2D eval_pose =
        dwb_core::projectPose(traj, lookahead_time_);
    end_yaw = eval_pose.theta;
  } else {
    end_yaw = traj.poses.back().theta;
  }

  return fabs(angles::shortest_angular_distance(end_yaw - cur_yaw_, goal_yaw_));
}

void KeepTargetInsightCritic::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!nav2_util::transformPoseInTargetFrame(
      *msg, latest_pose_, *tf_buffer_,
      "base_link"))
  {
    throw nav2_core::TFException("Transformed error in target updater node");
  }

  valid_data_ = true;
}

void KeepTargetInsightCritic::useCriticCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{

  if(request->data){
    // setScale(normal_sacle_);
    inuse_ = true;
    response->message = "Module KeepTargetInsight started successfully.";
  }else {
    // setScale(0.0);
    inuse_ = false;
    response->message = "Module KeepTargetInsight stopped successfully.";
  }

  response->success = true;
}

}  // namespace mcr_tracking_components
