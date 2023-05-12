// Copyright (c) 2021 Samsung Research America
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

#include <memory>
#include <string>
#include <vector>

#include "mcr_tracking_components/behavior_tree_nodes/compute_path_spline_poses_action.hpp"

namespace mcr_tracking_components
{

ComputePathSplinePosesAction::ComputePathSplinePosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<mcr_msgs::action::ComputePathSplinePoses>(xml_tag_name, action_name, conf)
{
}

void ComputePathSplinePosesAction::on_tick()
{
  getInput("poses", goal_.poses);
  getInput("planner_id", goal_.planner_id);
}

BT::NodeStatus ComputePathSplinePosesAction::on_success()
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
      return std::make_unique<mcr_tracking_components::ComputePathSplinePosesAction>(
        name, "compute_path_spline_poses", config);
    };

  factory.registerBuilder<mcr_tracking_components::ComputePathSplinePosesAction>(
    "ComputePathSplinePoses", builder);
}
