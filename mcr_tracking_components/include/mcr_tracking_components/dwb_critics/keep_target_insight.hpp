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
#ifndef mcr_tracking_components__KEEP_TARGET_INSIGHT_HPP_
#define mcr_tracking_components__KEEP_TARGET_INSIGHT_HPP_

#include <string>
#include <vector>
#include "dwb_core/trajectory_critic.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace mcr_tracking_components
{


class KeepTargetInsightCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  void reset() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

  virtual double scoreRotation(const dwb_msgs::msg::Trajectory2D & traj);

private:
  void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void useCriticCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  double lookahead_time_;
  double normal_sacle_;
  double goal_yaw_, cur_yaw_;
  bool valid_data_, inuse_;
};

}  // namespace mcr_tracking_components
#endif  // mcr_tracking_components__ROTATE_TO_GOAL_HPP_
