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
#ifndef MCR_PLANNER_PLUGINS__VON_NEUMANN_PATH_HPP_
#define MCR_PLANNER_PLUGINS__VON_NEUMANN_PATH_HPP_

#include <mcr_global_planner/traceback.hpp>

namespace mcr_planner_plugins
{
/**
 * @class VonNeumannPath
 * @brief Traceback function that moves from cell to cell using only the four neighbors
 */
class VonNeumannPath : public mcr_global_planner::Traceback
{
public:
  // Main Traceback interface
  nav_2d_msgs::msg::Path2D getPath(
    const mcr_global_planner::PotentialGrid & potential_grid,
    const geometry_msgs::msg::Pose2D & start,
    const geometry_msgs::msg::Pose2D & goal,
    double & path_cost) override;
};
}  // namespace mcr_planner_plugins

#endif  // MCR_PLANNER_PLUGINS__VON_NEUMANN_PATH_HPP_
