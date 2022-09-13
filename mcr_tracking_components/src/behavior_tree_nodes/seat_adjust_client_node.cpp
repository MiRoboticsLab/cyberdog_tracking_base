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

#include "mcr_tracking_components/behavior_tree_nodes/seat_adjust_client_node.hpp"

namespace mcr_tracking_components {

SeatAdjustClient::SeatAdjustClient(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<SeatAdjustT>(xml_tag_name, action_name, conf)
{
}

void SeatAdjustClient::on_wait_for_result()
{
  if (status() == BT::NodeStatus::SUCCESS) {
      // this->goal_done_ = true;
      RCLCPP_INFO(node_->get_logger(), "--------------on_wait_for_result SUCCESS --SeatAdjustClient");
  }
   RCLCPP_INFO(node_->get_logger(), "--------------on_wait_for_result --SeatAdjustClient");
}
void SeatAdjustClient::on_tick()
{
  if (!getInput("start", goal_.start)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "-----------------------------SeatAdjustClient: goal not provided");
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "------------------------------on_tick--SeatAdjustClient");
}
}
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<mcr_tracking_components::SeatAdjustClient>(
        name, "seatadjust", config);
    };

  factory.registerBuilder<mcr_tracking_components::SeatAdjustClient>(
    "SeatAdjustClient", builder);
}