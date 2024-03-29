// Copyright (c) 2018 Intel Corporation
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
#ifndef MCR_TRACKING_COMPONENTS__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
#define MCR_TRACKING_COMPONENTS__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_

#include <string>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace mcr_tracking_components
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::FollowPath
 */
class FollowPathAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowPath>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::FollowPathAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  FollowPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet
   */
  void on_wait_for_result() override;
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "goal to reach"),
        BT::InputPort<std::string>("controller_id", ""),
        BT::InputPort<std::string>("goal_checker_id", ""),
        BT::InputPort<std::string>("progress_checker_id", ""),
        BT::OutputPort<unsigned int>("output_exception_code", ""),
      });
  }
};

}  // namespace mcr_tracking_components

#endif  // MCR_TRACKING_COMPONENTS__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
