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

#ifndef MCR_GLOBAL_PLANNER__TRACEBACK_HPP_
#define MCR_GLOBAL_PLANNER__TRACEBACK_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "mcr_global_planner/potential.hpp"
#include "mcr_global_planner/cost_interpreter.hpp"
#include "mcr_nav_grid/nav_grid_info.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"

namespace mcr_global_planner
{
class Traceback
{
public:
  using Ptr = std::shared_ptr<Traceback>;
  Traceback()
  : cost_interpreter_(nullptr)
  {
  }
  virtual ~Traceback() = default;

  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr /*node*/,
    const std::string & name,
    CostInterpreter::Ptr cost_interpreter)
  {cost_interpreter_ = cost_interpreter; name_ = name;}

  virtual nav_2d_msgs::msg::Path2D getPath(
    const PotentialGrid & potential_grid,
    const geometry_msgs::msg::Pose2D & start,
    const geometry_msgs::msg::Pose2D & goal,
    double & path_cost) = 0;

protected:
  nav_2d_msgs::msg::Path2D mapPathToWorldPath(
    const nav_2d_msgs::msg::Path2D & original,
    const mcr_nav_grid::NavGridInfo & info)
  {
    nav_2d_msgs::msg::Path2D world_path;
    world_path.header.frame_id = info.frame_id;
    world_path.poses.resize(original.poses.size());
    for (unsigned int i = 0; i < original.poses.size(); i++) {
      world_path.poses[i].x = info.origin_x + original.poses[i].x * info.resolution;
      world_path.poses[i].y = info.origin_y + original.poses[i].y * info.resolution;
    }
    return world_path;
  }
  CostInterpreter::Ptr cost_interpreter_;
  std::string name_;
};
}  // namespace mcr_global_planner

#endif  // MCR_GLOBAL_PLANNER__TRACEBACK_HPP_
