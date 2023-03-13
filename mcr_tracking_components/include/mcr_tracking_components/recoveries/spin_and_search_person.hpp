// Copyright (c) 2018 Intel Corporation
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

#ifndef mcr_tracking_components__PLUGINS__SPIN_AND_SEARCH_HPP_
#define mcr_tracking_components__PLUGINS__SPIN_AND_SEARCH_HPP_

#include <chrono>
#include <string>
#include <memory>
#include "protocol/msg/person.hpp"
#include "nav2_recoveries/recovery.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace mcr_tracking_components
{
using SpinAction = nav2_msgs::action::Spin;

/**
 * @class mcr_tracking_components::Spin
 * @brief An action server recovery for spinning in
 */
class SpinAndSearchPerson : public nav2_recoveries::Recovery<SpinAction>
{
public:
  /**
   * @brief A constructor for mcr_tracking_components::SpinAndSearch
   */
  SpinAndSearchPerson();
  ~SpinAndSearchPerson();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of recovery
   */
  nav2_recoveries::Status onRun(const std::shared_ptr<const SpinAction::Goal> command) override;

  /**
   * @brief Configuration of recovery action
   */
  void onConfigure() override;

  /**
   * @brief Loop function to run behavior
   * @return Status of recovery
   */
  nav2_recoveries::Status onCycleUpdate() override;
  void callback_updated_goal(const protocol::msg::Person::SharedPtr msg);

protected:
  /**
   * @brief Check if pose is collision free
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   */
  bool isCollisionFree(
    const double & distance,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);

  rclcpp_lifecycle::LifecycleNode::SharedPtr shared_node_;
  SpinAction::Feedback::SharedPtr feedback_;
  rclcpp::Subscription<protocol::msg::Person>::SharedPtr goal_sub_;
  std::string goal_updater_topic_;
  rclcpp::Time latest_timestamped_;
  geometry_msgs::msg::PoseStamped latest_posestamped_;
  protocol::msg::Person latest_person_;

  double min_rotational_vel_;
  double max_rotational_vel_;
  double rotational_acc_lim_;
  double cmd_yaw_;
  double prev_yaw_;
  double relative_yaw_;
  double simulate_ahead_time_;
};

}  // namespace mcr_tracking_components

#endif  // NAV2_RECOVERIES__PLUGINS__SPIN_HPP_
