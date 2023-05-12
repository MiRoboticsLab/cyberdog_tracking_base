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

#ifndef mcr_tracking_components__PLUGINS__ACTION__TRACKING_MODE_DECIDER_NODE_HPP_
#define mcr_tracking_components__PLUGINS__ACTION__TRACKING_MODE_DECIDER_NODE_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "mcr_msgs/srv/switch_mode.hpp"
namespace mcr_tracking_components
{

/**
 * @brief A BT::ActionNodeBase to make decision for tracking mode
 */
class TrackingModeDecider : public BT::ActionNodeBase
{
public:
  /**
   * @brief A mcr_tracking_components::TrackingModeDecider constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TrackingModeDecider(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<unsigned char>("input_tracking_mode", "Original tracking mode"),
      BT::OutputPort<unsigned char>(
        "output_tracking_mode",
        "Output tracking mode by decision"),
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}
  void modeSwitchCallback(
    const std::shared_ptr<mcr_msgs::srv::SwitchMode::Request> request,
    std::shared_ptr<mcr_msgs::srv::SwitchMode::Response> response);
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  BT::NodeStatus tick() override;
  unsigned char last_mode_ = 255;
  rclcpp::Service<mcr_msgs::srv::SwitchMode>::SharedPtr service_;

};

}  // namespace mcr_tracking_components

#endif  // mcr_tracking_components__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
