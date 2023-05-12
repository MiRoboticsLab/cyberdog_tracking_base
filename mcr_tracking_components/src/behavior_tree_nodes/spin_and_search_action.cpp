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

#include <string>
#include <memory>

#include "mcr_tracking_components/behavior_tree_nodes/spin_and_search_action.hpp"

namespace mcr_tracking_components
{

SpinAndSearchAction::SpinAndSearchAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf)
{
  double dist;
  getInput("search_dist", dist);
  goal_.target_yaw = dist;
}

void SpinAndSearchAction::on_tick()
{
  increment_recovery_count();
}

}  // namespace mcr_tracking_components

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<mcr_tracking_components::SpinAndSearchAction>(
        name, "spin_and_search",
        config);
    };

  factory.registerBuilder<mcr_tracking_components::SpinAndSearchAction>("SpinAndSearch", builder);
}
