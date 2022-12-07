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

#ifndef mcr_tracking_components__PLUGINS__ACTION__SPIN_AND_SEARCH_ACTION_HPP_
#define mcr_tracking_components__PLUGINS__ACTION__SPIN_AND_SEARCH_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
namespace mcr_tracking_components
{

/**
 * @brief A mcr_tracking_components::BtActionNode class that wraps nav2_msgs::action::Spin
 */
class SpinAndSearchAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Spin>
{
public:
using MotionResultSrv = protocol::srv::MotionResultCmd;
  /**
   * @brief A constructor for mcr_tracking_components::SpinAndSearchAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  SpinAndSearchAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;
  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action. Could put a value on the blackboard.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  BT::NodeStatus on_success() override;
  /**
   * @brief Function to perform some user-defined operation whe the action is aborted.
   * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
   */
  BT::NodeStatus on_aborted() override;
  /**
   * @brief Function to perform some user-defined operation when the action is cancelled.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("search_dist", 1.57, "Spin distance")
      });
  }
protected:
  bool changeGait(float step_height);  
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<MotionResultSrv>::SharedPtr client_;  
};

}  // namespace mcr_tracking_components

#endif  // mcr_tracking_components__PLUGINS__ACTION__SPIN_ACTION_HPP_
