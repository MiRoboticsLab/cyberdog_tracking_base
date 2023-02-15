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
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "mcr_msgs/action/target_tracking.hpp"
#include "mcr_tracking_components/behavior_tree_nodes/tracking_mode_decider_node.hpp"

namespace mcr_tracking_components
{

TrackingModeDecider::TrackingModeDecider(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  service_ = node_->create_service<mcr_msgs::srv::SwitchMode>(
    "switch_mode",
    std::bind(
      &TrackingModeDecider::modeSwitchCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_default,
    callback_group_);

  // getInput("distance", distance_);
}
std::string dir_analysis(unsigned char dir)
{
  std::vector<std::string> dirs{"", "Behind", "Left", "Right", "Unkonwn"};
  if(dir > 3)dir = 4;
  return dirs[dir];
}

inline BT::NodeStatus TrackingModeDecider::tick()
{
  callback_group_executor_.spin_some();
  setStatus(BT::NodeStatus::RUNNING);
  unsigned char current_mode;

  getInput("input_tracking_mode", current_mode);
  if(current_mode > 3){
    current_mode = 1;
  }
  RCLCPP_INFO(node_->get_logger(), 
    "Next, use the %s tracking mode", dir_analysis(
    current_mode).c_str());
  setOutput("output_tracking_mode", current_mode);
  return BT::NodeStatus::SUCCESS;
}

void TrackingModeDecider::modeSwitchCallback(
  const std::shared_ptr<mcr_msgs::srv::SwitchMode::Request> request,
  std::shared_ptr<mcr_msgs::srv::SwitchMode::Response> response)
{
  config().blackboard->set<unsigned char>("relative_pos", request->relative_pos);
  std::cout << request->relative_pos << std::endl;
  response->result = true;
}

}  // namespace mcr_tracking_components

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mcr_tracking_components::TrackingModeDecider>("TrackingModeDecider");
}
