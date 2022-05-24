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


#ifndef MCR_NAV_GRID__COORDINATE_CONVERSION_HPP_
#define MCR_NAV_GRID__COORDINATE_CONVERSION_HPP_

#include <mcr_nav_grid/nav_grid_info.hpp>
#include <math.h>

namespace mcr_nav_grid
{

/**
 * @brief  Convert from grid coordinates to world coordinates of the center of the cell
 *
 * The resulting coordinates are for the center of the grid cell.
 *
 * @param[in]  mx The x grid coordinate
 * @param[in]  my The y grid coordinate
 * @param[out] wx Set to the associated x world coordinate
 * @param[out] wy Set to the associated y world coordinate
 */
inline void gridToWorld(const NavGridInfo & info, int mx, int my, double & wx, double & wy)
{
  wx = info.origin_x + (mx + 0.5) * info.resolution;
  wy = info.origin_y + (my + 0.5) * info.resolution;
}

/**
 * @brief  Convert from world coordinates to the precise (double) grid coordinates
 *
 * The results are not rounded, so that the values can be used for locating a position within a cell
 *
 * @param[in]  wx The x world coordinate
 * @param[in]  wy The y world coordinate
 * @param[out] mx Set to the associated x grid coordinate
 * @param[out] my Set to the associated y grid coordinate
 */
inline void worldToGrid(const NavGridInfo & info, double wx, double wy, double & mx, double & my)
{
  mx = (wx - info.origin_x) / info.resolution;
  my = (wy - info.origin_y) / info.resolution;
}

/**
 * @brief  Convert from world coordinates to grid coordinates without checking for legal bounds
 * @param[in]  wx The x world coordinate
 * @param[in]  wy The y world coordinate
 * @param[out] mx Set to the associated x grid coordinate
 * @param[out] my Set to the associated y grid coordinate
 * @note       The returned grid coordinates <b>are not guaranteed to lie within the grid.</b>
 */
inline void worldToGrid(const NavGridInfo & info, double wx, double wy, int & mx, int & my)
{
  double dmx, dmy;
  worldToGrid(info, wx, wy, dmx, dmy);
  mx = static_cast<int>(floor(dmx));
  my = static_cast<int>(floor(dmy));
}

/**
 * @brief  Convert from world coordinates to grid coordinates
 *
 * Combined functionality from costmap_2d::worldToMap and costmap_2d::worldToMapEnforceBounds.
 * The output parameters are set to grid indexes within the grid, even if the function returns false,
 * meaning the coordinates are outside the grid.
 *
 * @param[in]  wx The x world coordinate
 * @param[in]  wy The y world coordinate
 * @param[out] mx Set to the associated (bounds-enforced) x grid coordinate
 * @param[out] my Set to the associated (bounds-enforced) y grid coordinate
 * @return     True if the input coordinates were within the grid
 */
inline bool worldToGridBounded(
  const NavGridInfo & info, double wx, double wy, unsigned int & mx,
  unsigned int & my)
{
  double dmx, dmy;
  worldToGrid(info, wx, wy, dmx, dmy);

  bool valid = true;

  if (dmx < 0.0) {
    mx = 0;
    valid = false;
  } else if (dmx >= info.width) {
    mx = info.width - 1;
    valid = false;
  } else {
    mx = static_cast<unsigned int>(dmx);
  }

  if (dmy < 0.0) {
    my = 0;
    valid = false;
  } else if (dmy >= info.height) {
    my = info.height - 1;
    valid = false;
  } else {
    my = static_cast<unsigned int>(dmy);
  }

  return valid;
}

/**
 * @brief Check to see if the world coordinates are within the grid.
 *
 * This should only be used if the caller does not need the associated grid coordinates. Otherwise it would
 * be more efficient to call worldToGridBounded.
 *
 * @param[in]  wx The x world coordinate
 * @param[in]  wy The y world coordinate
 * @return     True if the input coordinates were within the grid
 */
inline bool isWithinGrid(const NavGridInfo & info, double wx, double wy)
{
  wx -= info.origin_x;
  wy -= info.origin_y;
  return wx >= 0.0 &&
         wy >= 0.0 &&
         wx < info.width * info.resolution &&
         wy < info.height * info.resolution;
}


}  // namespace mcr_nav_grid

#endif  // MCR_NAV_GRID__COORDINATE_CONVERSION_HPP_
