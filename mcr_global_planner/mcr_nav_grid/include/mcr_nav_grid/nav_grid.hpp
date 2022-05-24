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

#ifndef MCR_NAV_GRID__NAV_GRID_HPP_
#define MCR_NAV_GRID__NAV_GRID_HPP_

#include <mcr_nav_grid/nav_grid_info.hpp>
#include <mcr_nav_grid/index.hpp>
#include <string>

namespace mcr_nav_grid
{
/**
 * @class NavGrid
 * This class is a spiritual successor to the costmap_2d::Costmap2D class, with the key differences being that
 * the datatype and data storage methods are not specified, and the frame_id is specified.
 *
 * The templatized nature of the class allows you to store whatever you like at each grid location, including
 * unsigned chars if emulating Costmap2D or floating point numbers if emulating the grid_map package, or whatever
 * else.
 *
 * The VectorNavGrid class in this package implements this class with a straight-forward single-dimensional vector
 * representing the two dimensional grid. Other classes could implement the data storage differently.
 *
 * Getting data from the grid can be done either through the getValue methods or the parenthetical operators (which call
 * getValue internally). Implementing classes must implement getValue.
 * x = grid(0, 0) + grid.getValue(0, 1);
 *
 * Writing data to the grid must be done through the setValue method (which implementing classes must implement)
 * grid.setValue(0, 0, x);
 *
 * You can also use mcr_nav_grid::Index objects
 * mcr_nav_grid::Index index(0, 0);
 * x = grid(index) + grid.getValue(index);
 * index.y = 3;
 * grid.setCost(index, x);
 * The Index methods also internally call setValue/getValue
 *
 * The geometry of the grid is specified by the NavGridInfo. Borrowing an idea from the grid_map package, two
 * separate methods are defined for changing the info. setInfo will change the info without changing the grid values.
 * updateInfo will change the info while trying to preserve the contents of the grid.
 *
 * The final component is a collection of methods inspired by Costmap2D for converting coordinates of different types.
 */
template<typename T>
class NavGrid
{
public:
  explicit NavGrid(const T default_value = T {})
  : default_value_(default_value)
  {
  }

  /**
   * @brief Reset the contents of the grid
   */
  virtual void reset() = 0;

  /**
   * @brief get the value of the grid at (x,y)
   * @param x[in] Valid x coordinate
   * @param y[in] Valid y coordinate
   * @return value at (x,y)
   */
  virtual T getValue(const unsigned int x, const unsigned int y) const = 0;

  /**
   * @brief set the value of the grid at (x,y)
   * @param x[in] Valid x coordinate
   * @param y[in] Valid y coordinate
   * @param value[in] New Value
   */
  virtual void setValue(const unsigned int x, const unsigned int y, const T & value) = 0;

  /**@name Convenience Aliases */
  // Note: You may not be able to use these unless your deriving class declares
  // using NavGrid<T>::operator() or
  // using NavGrid<T>::getValue
  /**@{*/
  T getValue(const Index & index) {return getValue(index.x, index.y);}
  T operator()(const unsigned int x, const unsigned int y) const {return getValue(x, y);}
  T operator()(const Index & index) const {return getValue(index.x, index.y);}
  void setValue(const Index & index, const T & value) {setValue(index.x, index.y, value);}

  /**@}*/

  /**
   * @brief Change the info while attempting to keep the values associated with the grid coordinates
   * @param[in] new_info New grid info
   */
  virtual void setInfo(const NavGridInfo & new_info) = 0;

  /**
   * @brief Change the info while attempting to keep the values associated with the world coordinates
   *
   * For example, if the only change to the info is to the origin's x coordinate (increasing by an amount equal to the
   * resolution), then all the values should be shifted one grid cell to the left.
   *
   * @param[in] new_info New grid info
   */
  virtual void updateInfo(const NavGridInfo & new_info) {setInfo(new_info);}

  inline NavGridInfo getInfo() const {return info_;}

  /**
   * @brief Set the default value
   * @param[in] new_value New Default Value
   */
  void setDefaultValue(const T new_value)
  {
    default_value_ = new_value;
  }

  /*****************************************************************************************************
   * NavGridInfo accessor methods
   *****************************************************************************************************/
  inline unsigned int getWidth() const {return info_.width;}
  inline unsigned int getHeight() const {return info_.height;}
  inline double getResolution() const {return info_.resolution;}
  inline std::string getFrameId() const {return info_.frame_id;}
  inline double getOriginX() const {return info_.origin_x;}
  inline double getOriginY() const {return info_.origin_y;}

protected:
  NavGridInfo info_;
  T default_value_;
};

}  // namespace mcr_nav_grid

#endif  // MCR_NAV_GRID__NAV_GRID_HPP_
