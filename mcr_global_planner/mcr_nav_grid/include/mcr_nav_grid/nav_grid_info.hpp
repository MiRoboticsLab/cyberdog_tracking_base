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

#ifndef MCR_NAV_GRID__NAV_GRID_INFO_HPP_
#define MCR_NAV_GRID__NAV_GRID_INFO_HPP_

#include <string>

namespace mcr_nav_grid
{
/**
 * @struct NavGridInfo
 * This class defines a way to discretize a finite section of the world into a grid.
 * It contains similar information to the ROS msg nav_msgs/MapMetaData (aka the info field of nav_msgs/OccupancyGrid)
 * except the map_load_time is removed, the geometry is simplified from a Pose to xy coordinates, and the frame_id
 * is added.
 */
struct NavGridInfo
{
public:
  /* All data is publically accessible */
  unsigned int width = 0;
  unsigned int height = 0;
  double resolution = 1.0;
  std::string frame_id = "map";
  ///< The origin defines the coordinates of minimum corner of cell (0,0) in the grid
  double origin_x = 0.0;
  double origin_y = 0.0;

  /**
   * @brief comparison operator that requires all fields are equal
   */
  bool operator==(const NavGridInfo & info) const
  {
    return width == info.width && height == info.height && resolution == info.resolution &&
           origin_x == info.origin_x && origin_y == info.origin_y && frame_id == info.frame_id;
  }

  bool operator!=(const NavGridInfo & info) const
  {
    return !operator==(info);
  }

  /**
   * @brief String representation of this object
   */
  std::string toString() const
  {
    return std::to_string(width) + "x" + std::to_string(height) + " (" +
           std::to_string(resolution) + "res) " +
           frame_id + " " + std::to_string(origin_x) + " " + std::to_string(origin_y);
  }
};

inline std::ostream & operator<<(std::ostream & stream, const NavGridInfo & info)
{
  stream << info.toString();
  return stream;
}

}  // namespace mcr_nav_grid

#endif  // MCR_NAV_GRID__NAV_GRID_INFO_HPP_
