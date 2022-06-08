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
#ifndef MCR_PLANNER_PLUGINS__GRADIENT_PATH_HPP_
#define MCR_PLANNER_PLUGINS__GRADIENT_PATH_HPP_

#include <string>
#include "mcr_global_planner/traceback.hpp"
#include "mcr_nav_grid/vector_nav_grid.hpp"

namespace mcr_planner_plugins
{
/**
 * @class GradientPath
 * @brief Traceback function that creates a smooth gradient from the start to the goal
 */
class GradientPath : public mcr_global_planner::Traceback
{
public:
  // Main Traceback interface
  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & name,
    mcr_global_planner::CostInterpreter::Ptr cost_interpreter) override;

  nav_2d_msgs::msg::Path2D getPath(
    const mcr_global_planner::PotentialGrid & potential_grid,
    const geometry_msgs::msg::Pose2D & start,
    const geometry_msgs::msg::Pose2D & goal,
    double & path_cost) override;

protected:
  bool shouldGridStep(
    const mcr_global_planner::PotentialGrid & potential_grid,
    const mcr_nav_grid::Index & index);

  mcr_nav_grid::Index gridStep(
    const mcr_global_planner::PotentialGrid & potential_grid,
    const mcr_nav_grid::Index & index);

  inline void calculateGradient(
    const mcr_global_planner::PotentialGrid & potential_grid,
    const mcr_nav_grid::Index & index);

  double step_size_;
  double lethal_cost_;
  double iteration_factor_;
  bool grid_step_near_high_;
  mcr_nav_grid::VectorNavGrid<double> gradx_, grady_;
};
}  // namespace mcr_planner_plugins

#endif  // MCR_PLANNER_PLUGINS__GRADIENT_PATH_HPP_
