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

#ifndef MCR_GLOBAL_PLANNER__POTENTIAL_HPP_
#define MCR_GLOBAL_PLANNER__POTENTIAL_HPP_

#include <limits>
#include <memory>
#include "mcr_nav_grid/vector_nav_grid.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace mcr_global_planner
{
/**
 * Default value for potential indicating it has not yet been calculated
 */
const float HIGH_POTENTIAL = std::numeric_limits<float>::max();

typedef mcr_nav_grid::VectorNavGrid<float> PotentialGrid;


static inline mcr_nav_grid::NavGridInfo infoFromCostmap(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  mcr_nav_grid::NavGridInfo info;
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros->getCostmap();
  info.width = costmap->getSizeInCellsX();
  info.height = costmap->getSizeInCellsY();
  info.resolution = costmap->getResolution();
  info.frame_id = costmap_ros->getGlobalFrameID();
  info.origin_x = costmap->getOriginX();
  info.origin_y = costmap->getOriginY();
  return info;
}
}  // namespace mcr_global_planner
#endif  //  MCR_GLOBAL_PLANNER__POTENTIAL_HPP_
