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

#ifndef mcr_tracking_components__PLUGINS__ACTION__SEAT_ADJUST_CLIENT_NODE_HPP_
#define mcr_tracking_components__PLUGINS__ACTION__SEAT_ADJUST_CLIENT_NODE_HPP_

#include <memory>
#include <string>

// #include "behaviortree_cpp_v3/action_node.h"
#include "protocol/action/seat_adjust.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"


namespace mcr_tracking_components {

using SeatAdjustT = protocol::action::SeatAdjust;
/**
 * @brief A BT::ActionNodeBase to make decision for tracking mode
 */
class SeatAdjustClient : public nav2_behavior_tree::BtActionNode<SeatAdjustT> {
 public:
  /**
   * @brief A constructor for mcr_tracking_components::SpinAndSearchAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  SeatAdjustClient(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<unsigned int>("start", "To start the action."),
      });
  }

  virtual void on_wait_for_result() override;

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;
};

}  // namespace mcr_tracking_components

#endif  // mcr_tracking_components__PLUGINS__ACTION__CHANGE_GAIT_ACTION_HPP_
