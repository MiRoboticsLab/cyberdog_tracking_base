// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#include <string>
#include <memory>
#include "nav2_core/exceptions.hpp"
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "mcr_msgs/action/target_tracking.hpp"
#include <mcr_tracking_components/behavior_tree_nodes/charger_updater_node.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "Eigen/Dense"
#include "Eigen/QR"


namespace mcr_tracking_components
{

using std::placeholders::_1;
nav_msgs::msg::Path spline(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses);
ChargerUpdater::ChargerUpdater(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  distance_ = 2.0;
  std::string goal_updater_topic;

  nav2_util::declare_parameter_if_not_declared(
    node_, "global_frame",
    rclcpp::ParameterValue("map"));
  node_->get_parameter("global_frame", global_frame_);

  nav2_util::declare_parameter_if_not_declared(
    node_, "charger_updater_topic",
    rclcpp::ParameterValue("charger_pose"));
  node_->get_parameter("charger_updater_topic", goal_updater_topic);

  nav2_util::declare_parameter_if_not_declared(
    node_, "distance", rclcpp::ParameterValue(2.0));
  node_->get_parameter("distance", distance_);

  nav2_util::declare_parameter_if_not_declared(
    node_, "offset_x", rclcpp::ParameterValue(0.3));
  node_->get_parameter("offset_x", offset_x_);

  nav2_util::declare_parameter_if_not_declared(
    node_, "offset_y", rclcpp::ParameterValue(0.3));
  node_->get_parameter("offset_y", offset_y_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_updater_topic, rclcpp::SensorDataQoS(),
    std::bind(&ChargerUpdater::callback_updated_goal, this, _1),
    sub_option);

  transformed_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    goal_updater_topic + "_transformed", rclcpp::SensorDataQoS());

  historical_poses_.clear();

  last_goal_received_.header.frame_id = global_frame_;
}

inline BT::NodeStatus ChargerUpdater::tick()
{
  callback_group_executor_.spin_some();
  std::lock_guard<std::mutex> guard(mutex_);
  geometry_msgs::msg::PoseStamped goal;
  setOutput("output_exception_code", nav2_core::NOEXCEPTION);

  if ((node_->now().seconds() - latest_timestamp_.seconds()) > 1.5) {
    RCLCPP_WARN(node_->get_logger(), "The target pose may be lost.");
    historical_poses_.clear();
    setOutput("output_exception_code", nav2_core::DETECTOREXCEPTION);
    config().blackboard->set<int>("exception_code", nav2_core::DETECTOREXCEPTION);
    return BT::NodeStatus::FAILURE;
  }

  if (rclcpp::Time(last_goal_received_.header.stamp) > rclcpp::Time(goal.header.stamp)) {
    goal = last_goal_received_;
    // goal = historical_poses_.front();
  }

  if (historical_poses_.size() >= 1) {
    setOutput(
      "output_goals",
      std::vector<geometry_msgs::msg::PoseStamped>(
        historical_poses_.rbegin(),
        historical_poses_.rend()));
  }


  BT::NodeStatus status = child_node_->executeTick();

  if (status == BT::NodeStatus::FAILURE) {
    setOutput("output_exception_code", nav2_core::PLANNEREXECPTION);
    config().blackboard->set<int>("exception_code", nav2_core::PLANNEREXECPTION);
  }

  return status;
}


void
ChargerUpdater::callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> guard(mutex_);
  latest_timestamp_ = node_->now();

  geometry_msgs::msg::PoseStamped msg_with_orientation = *msg;
  msg_with_orientation.pose.position.x += offset_x_;
  msg_with_orientation.pose.position.y += offset_y_;

  //该模式下的跟随位姿对应在全局坐标系上的位姿
  geometry_msgs::msg::PoseStamped global_frame_pose, global_frame_pose_b;

  if (!nav2_util::transformPoseInTargetFrame(
      msg_with_orientation, global_frame_pose, *tf_buffer_,
      global_frame_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Faild transform target pose to %s", global_frame_.c_str());
    throw nav2_core::TFException("Transformed error in target updater node");
  }

  //visu debug info
  if (transformed_pose_pub_ != nullptr &&
    node_->count_subscribers(transformed_pose_pub_->get_topic_name()) > 0)
  {
    transformed_pose_pub_->publish(global_frame_pose);
  }

  global_frame_pose_b = global_frame_pose;
  global_frame_pose_b.pose.position.x -= 2.0;

  geometry_msgs::msg::PoseStamped pose_based_on_global_frame;

  if (!nav2_util::getCurrentPose(pose_based_on_global_frame, *tf_buffer_, global_frame_)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to obtain current pose based on map coordinate system.");
    return;
  }


  historical_poses_.clear();
  historical_poses_.push_front(std::move(pose_based_on_global_frame));
  historical_poses_.push_front(std::move(global_frame_pose_b));
  historical_poses_.push_front(std::move(global_frame_pose));
}

}  // namespace mcr_tracking_components

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mcr_tracking_components::ChargerUpdater>("ChargerUpdater");
}
