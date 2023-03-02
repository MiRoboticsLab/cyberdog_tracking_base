// Copyright (c) 2021 Samsung Research
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

#ifndef bt_navigators__NAVIGATORS__TARGET_TRACKING_HPP_
#define bt_navigators__NAVIGATORS__TARGET_TRACKING_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "bt_navigators/navigator.hpp"
#include "mcr_msgs/action/target_tracking.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace bt_navigators
{

/**
 * @class TargetTrackingNavigator
 * @brief A navigator for tracking target
 */
class TargetTrackingNavigator
  : public bt_navigators::Navigator<mcr_msgs::action::TargetTracking>
{
public:
  using ActionT = mcr_msgs::action::TargetTracking;

  /**
   * @brief A constructor for TargetTrackingNavigator
   */
  TargetTrackingNavigator()
  : Navigator() {}

  /**
   * @brief A configure state transition to configure navigator's state
   * @param node Weakptr to the lifecycle node
   */
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  /**
   * @brief A cleanup state transition to remove memory allocated
   */
  bool cleanup() override;

  /**
   * @brief A subscription and callback to handle the topic-based goal published
   * from rviz
   * @param pose Pose received via atopic
   */
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  /**
   * @brief Get action name for this navigator
   * @return string Name of action server
   */
  std::string getName() {return std::string("tracking_target");}

  /**
   * @brief Get navigator's default BT
   * @param node WeakPtr to the lifecycle node
   * @return string Filepath to default XML
   */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   * @param goal Action template's goal message
   * @return bool if goal was received successfully to be processed
   */
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  void onLoop() override;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that is called when a the action is completed, can fill in
   * action result message or indicate that this action is done.
   * @param result Action template result message to populate
   */
  void goalCompleted(typename ActionT::Result::SharedPtr result) override;

  /**
   * @brief Goal pose initialization on the blackboard
   * @param goal Action template's goal message to process
   */
  void initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string goal_blackboard_relative_pos_;
  std::string goal_blackboard_keep_dist_;
  std::string target_topic_;
  double maxwait_;
  geometry_msgs::msg::PoseStamped latest_goal_;
  // Odometry smoother object
  std::unique_ptr<nav2_util::OdomSmoother> odom_smoother_;

  bool onGoalUpdate(FollowPoses::SharedPtr msg);
};

}  // namespace bt_navigators

#endif  // bt_navigators__NAVIGATORS__TARGET_TRACKING_HPP_