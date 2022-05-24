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

#ifndef MCR_PLANNER_PLUGINS__DIJKSTRA_HPP_
#define MCR_PLANNER_PLUGINS__DIJKSTRA_HPP_

#include <mcr_global_planner/potential_calculator.hpp>
#include <queue>

namespace mcr_planner_plugins
{
/**
 * @class Dijkstra
 * @brief Potential calculator that explores the potential breadth first while using the kernel function
 */
class Dijkstra : public mcr_global_planner::PotentialCalculator
{
public:
  // Main PotentialCalculator interface
  unsigned int updatePotentials(
    mcr_global_planner::PotentialGrid & potential_grid,
    const geometry_msgs::msg::Pose2D & start,
    const geometry_msgs::msg::Pose2D & goal) override;

protected:
  /**
   * @brief Calculate the potential for next_index if not calculated already
   *
   * @param potential_grid Potential grid
   * @param next_index Coordinates of cell to calculate
   */
  void add(mcr_global_planner::PotentialGrid & potential_grid, mcr_nav_grid::Index next_index);

  std::queue<mcr_nav_grid::Index> queue_;
};
}  // namespace mcr_planner_plugins
#endif  // MCR_PLANNER_PLUGINS__DIJKSTRA_HPP_
