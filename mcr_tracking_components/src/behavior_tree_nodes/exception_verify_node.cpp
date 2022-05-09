// Copyright (c) 2021 Samsung Research America
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

#include <chrono>
#include <string>

#include "mcr_tracking_components/behavior_tree_nodes/exception_verify_node.hpp"

namespace mcr_tracking_components
{

ExceptionVerify::ExceptionVerify(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");  
}

BT::NodeStatus ExceptionVerify::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  unsigned int a, b;
  getInput("exception_code", a);
  getInput("expected_code", b);
  if (a != b) {
    return BT::NodeStatus::FAILURE;
  }
  return child_node_->executeTick();
}

}  // namespace mcr_tracking_components

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mcr_tracking_components::ExceptionVerify>("ExceptionVerify");
}
