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

#include <memory>
#include <string>

#include "mcr_tracking_components/behavior_tree_nodes/follow_path_action.hpp"
#include "nav2_core/exceptions.hpp"
namespace mcr_tracking_components
{

FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{
}

void FollowPathAction::on_tick()
{
  getInput("path", goal_.path);
  getInput("goal", goal_.goal);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);
  getInput("progress_checker_id", goal_.progress_checker_id);
}

void FollowPathAction::on_wait_for_result()
{
  // Grab the new path
  nav_msgs::msg::Path new_path;
  int exception_code = 0;
  getInput("path", new_path);
  config().blackboard->get<int>("exception_code", exception_code);

  // Check if it is not same with the current one
  if (goal_.path != new_path) {
    // the action server on the next loop iteration
    goal_.path = new_path;
    if(exception_code == nav2_core::CONTROLLEREXECPTION){
      config().blackboard->set<int>("exception_code", nav2_core::NOEXCEPTION);
    }
    goal_updated_ = true;
  }
  geometry_msgs::msg::PoseStamped new_goal;
  getInput("goal", new_goal);

  // Check if it is not same with the current one
  if (goal_.goal != new_goal) {
    // the action server on the next loop iteration
    goal_.goal = new_goal;
    goal_updated_ = true;
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);

  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }
  std::string new_progress_checker_id;
  getInput("progress_checker_id", new_progress_checker_id);
  if (goal_.progress_checker_id != new_progress_checker_id) {
    goal_.progress_checker_id = new_progress_checker_id;
    goal_updated_ = true;
  }
}

BT::NodeStatus FollowPathAction::on_aborted()
{
  config().blackboard->set<int>("exception_code", nav2_core::CONTROLLEREXECPTION);
  setOutput("output_exception_code", nav2_core::CONTROLLEREXECPTION);
  return BT::NodeStatus::FAILURE;
}  

}  // namespace mcr_tracking_components

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<mcr_tracking_components::FollowPathAction>(
        name, "follow_p", config);
    };

  factory.registerBuilder<mcr_tracking_components::FollowPathAction>(
    "FollowP", builder);
}
