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

#include "mcr_tracking_components/behavior_tree_nodes/change_gait_node.hpp"

#include <unistd.h>

#include <limits>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mcr_msgs/action/target_tracking.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mcr_tracking_components {

ChangeGait::ChangeGait(const std::string& name,
                       const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_,
                                              node_->get_node_base_interface());
  motion_client_ = node_->create_client<MotionServiceT>("motion_result_cmd");
  change_gait_pub_ =
      node_->create_publisher<std_msgs::msg::Int8>("motion_control", 10);
}
inline BT::NodeStatus ChangeGait::tick() {
  callback_group_executor_.spin_some();
  setStatus(BT::NodeStatus::RUNNING);
  int current_gait;

  getInput("gait", current_gait);

  // if (!motion_client_->service_is_ready()) {
  //   RCLCPP_ERROR(node_->get_logger(), "ChangeGait error: motion service is
  //   not ready."); return BT::NodeStatus::FAILURE;
  // }

  // auto request = std::make_shared<MotionServiceT::Request>();
  // request->motion_id = 8; // 恢复站立
  // auto motion_future = motion_client_->async_send_request(request);

  // auto timeout = std::chrono::milliseconds(1000);
  // if (motion_future.wait_for(timeout) == std::future_status::ready) {
  //   auto response = motion_future.get();
  //   if (response->result == true) {
  //     RCLCPP_INFO(node_->get_logger(), "ChangeGait ok: motion gait now is
  //     %d.", request->motion_id); return BT::NodeStatus::SUCCESS;
  //   } else {
  //     RCLCPP_ERROR(node_->get_logger(), "ChangeGait failed: respose is not
  //     ok."); return BT::NodeStatus::FAILURE;
  //   }
  // }
  // RCLCPP_ERROR(node_->get_logger(), "ChangeGait failed: didn't wait for
  // respose."); return BT::NodeStatus::FAILURE;
  std_msgs::msg::Int8 gait;
  gait.data = 3;
  change_gait_pub_->publish(gait);
  RCLCPP_INFO(node_->get_logger(), "ChangeGait ok: motion gait now is %d.",
              gait.data);

  if (current_gait == 7)
    sleep(5);
  else
    sleep(1);

  gait.data = current_gait;
  change_gait_pub_->publish(gait);
  RCLCPP_INFO(node_->get_logger(), "ChangeGait ok: motion gait now is %d.",
              gait.data);
  sleep(2);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mcr_tracking_components

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<mcr_tracking_components::ChangeGait>("ChangeGait");
}
