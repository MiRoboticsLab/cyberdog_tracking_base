// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
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

#include "mcr_tracking_components/behavior_tree_nodes/is_battery_in_charge_condition.hpp"

#include <string>

namespace mcr_tracking_components {

IsBatteryInChargeCondition::IsBatteryInChargeCondition(
    const std::string& condition_name, const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf),
      battery_topic_("/battery_status"),
      min_battery_(0.0),
      is_voltage_(false),
      is_battery_low_(false) {
  getInput("min_battery", min_battery_);
  getInput("battery_topic", battery_topic_);
  getInput("is_voltage", is_voltage_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_,
                                              node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&IsBatteryInChargeCondition::batteryCallback, this,
                std::placeholders::_1),
      sub_option);
}

BT::NodeStatus IsBatteryInChargeCondition::tick() {
  callback_group_executor_.spin_some();
  // if (is_battery_low_) {
  //   return BT::NodeStatus::SUCCESS;
  // }
  // return BT::NodeStatus::FAILURE;
  RCLCPP_INFO(node_->get_logger(), "Recharge successful.");
  return BT::NodeStatus::SUCCESS;
}

void IsBatteryInChargeCondition::batteryCallback(
    sensor_msgs::msg::BatteryState::SharedPtr msg) {
  if (is_voltage_) {
    is_battery_low_ = msg->voltage <= min_battery_;
  } else {
    is_battery_low_ = msg->percentage <= min_battery_;
  }
}

}  // namespace mcr_tracking_components

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<mcr_tracking_components::IsBatteryInChargeCondition>(
      "IsBatteryInCharge");
}