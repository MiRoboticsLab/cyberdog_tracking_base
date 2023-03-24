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

#include "mcr_tracking_components/behavior_tree_nodes/compute_path_to_pose_action.hpp"

namespace mcr_tracking_components
{

ComputePathToPoseAction::ComputePathToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathToPose>(xml_tag_name, action_name, conf)
{
}

void ComputePathToPoseAction::on_tick()
{
  getInput("goal", goal_.goal);
  getInput("planner_id", goal_.planner_id);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
  goal_updated_ = true;
}

void ComputePathToPoseAction::on_wait_for_result()
{
  // geometry_msgs::msg::PoseStamped goal; 
  // getInput("goal", goal);
  // if (goal_.goal != goal) {
  //   goal_.goal = goal;
  //   goal_updated_ = true;
  // }
}

BT::NodeStatus ComputePathToPoseAction::on_success()
{
  setOutput("path", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mcr_tracking_components

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<mcr_tracking_components::ComputePathToPoseAction>(
        name, "compute_path_to_p", config);
    };

  factory.registerBuilder<mcr_tracking_components::ComputePathToPoseAction>(
    "ComputePathToP", builder);
}
