// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef mcr_tracking_components__KEEP_PERSON_INSIGHT_HPP_
#define mcr_tracking_components__KEEP_PERSON_INSIGHT_HPP_

#include <string>
#include <vector>
#include "dwb_core/trajectory_critic.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "protocol/msg/person.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace mcr_tracking_components
{


class KeepPersonInsightCritic : public dwb_core::TrajectoryCritic
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
  void poseCallback(protocol::msg::Person::SharedPtr msg);
  // rclcpp::Node::SharedPtr rcl_node_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> rcl_node_;
  rclcpp::Subscription<protocol::msg::Person>::SharedPtr pose_sub_;
  protocol::msg::Person latest_person_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double lookahead_time_;
  rclcpp::Time latest_timestamp_;
  double normal_sacle_;
  double goal_yaw_, cur_yaw_;
  bool inuse_;
  std::string target_topic_;
};

}  // namespace mcr_tracking_components
#endif  // mcr_tracking_components__KEEP_PERSON_INSIGHT_HPP_
