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

#include <string>
#include <memory>

#include "mcr_tracking_components/behavior_tree_nodes/spin_and_search_action.hpp"

namespace mcr_tracking_components
{

SpinAndSearchAction::SpinAndSearchAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf)
{
  double dist;
  getInput("search_dist", dist);
  goal_.target_yaw = dist;
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<MotionResultSrv>("velocity_adaptor_gait");
}

using namespace std::chrono_literals;
bool SpinAndSearchAction::changeGait(float step_height)
{
  auto request = std::make_shared<MotionResultSrv::Request>();
  request->motion_id = 309;
  request->value = 0;
  request->step_height = std::vector<float>{step_height, step_height};
  request->cmd_source = 4;

  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client_->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "Succeed to change gait to %lf", step_height);
    return true;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to change gait to %lf", step_height);
    return false;
  }
}
void SpinAndSearchAction::on_tick()
{
  increment_recovery_count();
  changeGait(0.01);
}
BT::NodeStatus SpinAndSearchAction::on_success()
{
  changeGait(0.06);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SpinAndSearchAction::on_aborted()
{
  changeGait(0.06);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SpinAndSearchAction::on_cancelled()
{
  changeGait(0.06);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace mcr_tracking_components

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<mcr_tracking_components::SpinAndSearchAction>(
        name, "spin_and_search",
        config);
    };

  factory.registerBuilder<mcr_tracking_components::SpinAndSearchAction>("SpinAndSearch", builder);
}
