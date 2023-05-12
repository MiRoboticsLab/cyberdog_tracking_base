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

#ifndef mcr_tracking_components__PLUGINS__ACTION__CHANGE_GAIT_NODE_HPP_
#define mcr_tracking_components__PLUGINS__ACTION__CHANGE_GAIT_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
namespace mcr_tracking_components {

using MotionServiceT = protocol::srv::MotionResultCmd;
/**
 * @brief A BT::BtServiceNode to make decision for tracking mode
 */
class ChangeGait : public nav2_behavior_tree::BtServiceNode<MotionServiceT> {
 public:
  /**
   * @brief A mcr_tracking_components::ChangeGait constructor
   * @param name Name for this node
   * @param conf BT node configuration
   */
  ChangeGait(const std::string& name,
             const BT::NodeConfiguration& conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {
    return providedBasicPorts(
      {
        BT::InputPort<int>("gait_motion_id", "the gait for next step."),
        BT::InputPort<float>("gait_shape_value", "the gait for next step."),
        BT::InputPort<float>("gait_step_height", "the gait for next step."),
        BT::OutputPort<unsigned int>("next_action_start",
                              "start signal to next action."),
      });
  }

  virtual void on_wait_for_result();

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  virtual void on_tick();
};

}  // namespace mcr_tracking_components

#endif  // mcr_tracking_components__PLUGINS__ACTION__CHANGE_GAIT_ACTION_HPP_
