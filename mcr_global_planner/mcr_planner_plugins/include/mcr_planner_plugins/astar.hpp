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

#ifndef MCR_PLANNER_PLUGINS__ASTAR_HPP_
#define MCR_PLANNER_PLUGINS__ASTAR_HPP_


#include <queue>
#include <vector>
#include <string>
#include <memory>
#include "mcr_global_planner/potential_calculator.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mcr_planner_plugins
{

class AStar : public mcr_global_planner::PotentialCalculator
{
public:
  // Main PotentialCalculator interface
  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    mcr_global_planner::CostInterpreter::Ptr cost_interpreter) override;

  unsigned int updatePotentials(
    mcr_global_planner::PotentialGrid & potential_grid,
    const geometry_msgs::msg::Pose2D & start,
    const geometry_msgs::msg::Pose2D & goal) override;

protected:
  /**
   * @brief Calculate the potential for index if not calculated already
   *
   * @param potential_grid Potential grid
   * @param prev_potential Potential of the previous cell
   * @param index Coordinates of cell to calculate
   * @param start_index Coordinates of start cell (for heuristic calculation)
   */
  void add(
    mcr_global_planner::PotentialGrid & potential_grid, double prev_potential,
    const mcr_nav_grid::Index & index, const mcr_nav_grid::Index & start_index);

  /**
   * @brief Calculate the heuristic value for a particular cell
   *
   * @param index Coordinates of cell to calculate
   * @param start_index Coordinates of start cell
   */
  float getHeuristicValue(
    const mcr_nav_grid::Index & index,
    const mcr_nav_grid::Index & start_index) const;

  /**
   * @brief Helper Class for sorting indexes by their heuristic
   */
  struct QueueEntry
  {
public:
    QueueEntry(mcr_nav_grid::Index index, float heuristic)
    : i(index), cost(heuristic)
    {
    }
    mcr_nav_grid::Index i;
    float cost;
  };

  /**
   * @brief Comparator for sorting the QueueEntrys
   */
  struct QueueEntryComparator
  {
    bool operator()(const QueueEntry & a, const QueueEntry & b) const
    {
      return a.cost > b.cost;
    }
  };

  // Indexes sorted by heuristic
  using AStarQueue = std::priority_queue<QueueEntry, std::vector<QueueEntry>,
      QueueEntryComparator>;
  AStarQueue queue_;
  bool manhattan_heuristic_;
  bool use_kernel_;
  double minimum_requeue_change_;
};
}  // namespace mcr_planner_plugins


#endif  // MCR_PLANNER_PLUGINS__ASTAR_HPP_
