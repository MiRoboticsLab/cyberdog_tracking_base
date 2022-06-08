// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef MCR_GLOBAL_PLANNER__SPLINER_HPP_
#define MCR_GLOBAL_PLANNER__SPLINER_HPP_

#include <limits>
#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"

namespace mcr_global_planner
{
class Spliner
{
public:
  using Ptr = std::shared_ptr<Spliner>;
  Spliner()
  {
  }
  virtual ~Spliner() = default;
  virtual nav_msgs::msg::Path spline(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses) = 0;
  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr /*node*/,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>     /*costmap*/
  ) = 0;

  std::string name_;
};
}  // namespace mcr_global_planner
#endif  // MCR_GLOBAL_PLANNER__SPLINER_HPP_
