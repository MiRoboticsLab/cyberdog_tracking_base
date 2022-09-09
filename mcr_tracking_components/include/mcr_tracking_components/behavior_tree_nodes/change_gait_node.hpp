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

#ifndef mcr_tracking_components__PLUGINS__ACTION__CHANGE_GAIT_NODE_HPP_
#define mcr_tracking_components__PLUGINS__ACTION__CHANGE_GAIT_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
namespace mcr_tracking_components {

using MotionServiceT = protocol::srv::MotionResultCmd;
/**
 * @brief A BT::ActionNodeBase to make decision for tracking mode
 */
class ChangeGait : public BT::ActionNodeBase {
 public:
  /**
   * @brief A mcr_tracking_components::ChangeGait constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ChangeGait(const std::string& xml_tag_name,
             const BT::NodeConfiguration& conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("gait", "the gait for next step."),
        BT::OutputPort<unsigned int>("next_action_start",
                              "start signal to next action."),
    };
  }

 private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  BT::NodeStatus tick() override;
  rclcpp::Client<MotionServiceT>::SharedPtr motion_client_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr change_gait_pub_;
};

}  // namespace mcr_tracking_components

#endif  // mcr_tracking_components__PLUGINS__ACTION__CHANGE_GAIT_ACTION_HPP_
