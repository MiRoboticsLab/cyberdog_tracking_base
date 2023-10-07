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

#include <memory>
#include <string>
#include <limits>
#include "mcr_tracking_components/controller_plugins/unreachable_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace mcr_tracking_components
{

UnreachableGoalChecker::UnreachableGoalChecker()
: xy_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  stateful_(true),
  check_xy_(true),
  xy_goal_tolerance_sq_(0.0625)
{
}

void UnreachableGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

 // Setup callback for changes to parameters.
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  parameter_event_sub_ = parameters_client_->on_parameter_event(
    std::bind(&UnreachableGoalChecker::on_parameter_event_callback, this, _1));
}

void UnreachableGoalChecker::reset()
{
}

bool UnreachableGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & , const geometry_msgs::msg::Pose & ,
  const geometry_msgs::msg::Twist &)
{
  return false;
}

bool UnreachableGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & ,
  geometry_msgs::msg::Twist & )
{
  return true;
}

void
UnreachableGoalChecker::on_parameter_event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr )
{
}

}  // namespace mcr_tracking_components

PLUGINLIB_EXPORT_CLASS(mcr_tracking_components::UnreachableGoalChecker, nav2_core::GoalChecker)
