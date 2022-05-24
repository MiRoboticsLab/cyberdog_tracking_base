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
#include <cmath>
#include <string>
#include <memory>
#include "mcr_planner_plugins/astar.hpp"
#include "mcr_global_planner/exceptions.hpp"

#include "mcr_nav_grid/coordinate_conversion.hpp"
#include "mcr_global_planner/kernel_function.hpp"
#include "nav2_util/node_utils.hpp"


namespace mcr_planner_plugins
{
void AStar::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap*/,
  mcr_global_planner::CostInterpreter::Ptr cost_interpreter)
{
  cost_interpreter_ = cost_interpreter;
  name_ = name;
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".manhattan_heuristic", rclcpp::ParameterValue(
      false));
  node->get_parameter(name_ + ".manhattan_heuristic", manhattan_heuristic_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_kernel",
    rclcpp::ParameterValue(true));
  node->get_parameter(name + ".use_kernel", use_kernel_);
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".minimum_requeue_change", rclcpp::ParameterValue(
      1.0));
  node->get_parameter(name_ + ".minimum_requeue_change", minimum_requeue_change_);
}

unsigned int AStar::updatePotentials(
  mcr_global_planner::PotentialGrid & potential_grid,
  const geometry_msgs::msg::Pose2D & start,
  const geometry_msgs::msg::Pose2D & goal)
{
  const mcr_nav_grid::NavGridInfo & info = potential_grid.getInfo();
  queue_ = AStarQueue();
  potential_grid.reset();

  mcr_nav_grid::Index goal_i;
  worldToGridBounded(info, goal.x, goal.y, goal_i.x, goal_i.y);
  queue_.push(QueueEntry(goal_i, 0.0));
  potential_grid.setValue(goal_i, 0.0);

  // bounds check done in mcr_global_planner
  mcr_nav_grid::Index start_i;
  worldToGridBounded(info, start.x, start.y, start_i.x, start_i.y);

  if (potential_grid.getWidth() == 0 || potential_grid.getHeight() == 0) {
    return 0;
  }

  unsigned int width_bound = potential_grid.getWidth() - 1,
    height_bound = potential_grid.getHeight() - 1;
  unsigned int c = 0;

  while (queue_.size() > 0) {
    QueueEntry top = queue_.top();
    queue_.pop();
    c++;

    mcr_nav_grid::Index i = top.i;
    if (i == start_i) {return c;}

    double prev_potential = potential_grid(i);

    if (i.x < width_bound) {
      add(potential_grid, prev_potential, mcr_nav_grid::Index(i.x + 1, i.y), start_i);
    }
    if (i.x > 0) {
      add(potential_grid, prev_potential, mcr_nav_grid::Index(i.x - 1, i.y), start_i);
    }
    if (i.y < height_bound) {
      add(potential_grid, prev_potential, mcr_nav_grid::Index(i.x, i.y + 1), start_i);
    }
    if (i.y > 0) {
      add(potential_grid, prev_potential, mcr_nav_grid::Index(i.x, i.y - 1), start_i);
    }
  }

  throw mcr_global_planner::NoGlobalPathException();
}

void AStar::add(
  mcr_global_planner::PotentialGrid & potential_grid, double prev_potential,
  const mcr_nav_grid::Index & index, const mcr_nav_grid::Index & start_index)
{
  float cost = cost_interpreter_->getCost(index.x, index.y);
  if (cost_interpreter_->isLethal(cost)) {
    return;
  }

  float new_potential;
  if (use_kernel_) {
    new_potential = mcr_global_planner::calculateKernel(potential_grid, cost, index.x, index.y);
  } else {
    new_potential = prev_potential + cost;
  }

  if (new_potential >= potential_grid(index) ||
    potential_grid(index) - new_potential < minimum_requeue_change_)
  {
    return;
  }

  potential_grid.setValue(index, new_potential);
  queue_.push(QueueEntry(index, new_potential + getHeuristicValue(index, start_index)));
}

inline unsigned int uintDiff(const unsigned int a, const unsigned int b)
{
  return (a > b) ? a - b : b - a;
}

float AStar::getHeuristicValue(
  const mcr_nav_grid::Index & index,
  const mcr_nav_grid::Index & start_index) const
{
  unsigned int dx = uintDiff(start_index.x, index.x);
  unsigned int dy = uintDiff(start_index.y, index.y);
  float distance;
  if (manhattan_heuristic_) {
    distance = static_cast<float>(dx + dy);
  } else {
    distance = hypot(dx, dy);
  }
  return distance * cost_interpreter_->getNeutralCost();
}

}  // namespace mcr_planner_plugins
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mcr_planner_plugins::AStar, mcr_global_planner::PotentialCalculator)
