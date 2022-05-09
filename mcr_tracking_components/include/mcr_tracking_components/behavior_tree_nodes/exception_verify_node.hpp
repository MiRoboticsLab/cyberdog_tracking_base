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

#ifndef mcr_tracking_components__PLUGINS__DECORATOR__EXCEPTION_VERIFY_NODE_HPP_
#define mcr_tracking_components__PLUGINS__DECORATOR__EXCEPTION_VERIFY_NODE_HPP_

#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

namespace mcr_tracking_components
{

/**
 * @brief A BT::DecoratorNode that triggers its child only once and returns FAILURE
 * for every succeeding tick
 */
class ExceptionVerify : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for mcr_tracking_components::ExceptionVerify
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ExceptionVerify(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<unsigned int>("exception_code", 0, "Exception code for check."),
      BT::InputPort<unsigned int>("expected_code", 0, "Expected code for check."),
    };    
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace mcr_tracking_components

#endif  // mcr_tracking_components__PLUGINS__DECORATOR__SINGLE_TRIGGER_NODE_HPP_
