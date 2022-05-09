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
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"
#include "nav2_util/robot_utils.hpp"

PLUGINLIB_EXPORT_CLASS(mcr_tracking_components::KeepTargetInsightCritic, dwb_core::TrajectoryCritic)

namespace mcr_tracking_components
{

inline double hypot_sq(double dx, double dy)
{
  return dx * dx + dy * dy;
}

void KeepTargetInsightCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  lookahead_time_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".lookahead_time", -1.0);

  pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/tracking_pose",
    rclcpp::SensorDataQoS(),
    std::bind(&KeepTargetInsightCritic::poseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(
    node->get_logger(), "Keep target insight critic subscribed to tracking pose: tracking_pose");

  reset();
}

void KeepTargetInsightCritic::reset()
{
  valid_data_ = false;
}

bool KeepTargetInsightCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & /*vel*/,
  const geometry_msgs::msg::Pose2D & /*goal*/,
  const nav_2d_msgs::msg::Path2D &)
{
  goal_yaw_ = atan2(latest_pose_.pose.position.y, latest_pose_.pose.position.x);
  cur_yaw_ = pose.theta;
  return true;
}

double KeepTargetInsightCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  // If we're not sufficiently close to the goal, we don't care what the twist is
  if (!valid_data_) {
    return 0.0;
  }
  return scoreRotation(traj);
}

double KeepTargetInsightCritic::scoreRotation(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (traj.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory.");
  }

  double end_yaw;
  if (lookahead_time_ >= 0.0) {
    geometry_msgs::msg::Pose2D eval_pose = dwb_core::projectPose(traj, lookahead_time_);
    end_yaw = eval_pose.theta;
  } else {
    end_yaw = traj.poses.back().theta;
  }

  return fabs(angles::shortest_angular_distance(end_yaw - cur_yaw_, goal_yaw_));
}

void KeepTargetInsightCritic::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_pose_ = *msg;
  valid_data_ = true;
}
}  // namespace mcr_tracking_components
