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

#ifndef MCR_NAV_GRID__INDEX_HPP_
#define MCR_NAV_GRID__INDEX_HPP_

#include <string>

namespace mcr_nav_grid
{
/**
 * @class GenericIndex
 * @brief A simple pair of x/y coordinates
 */
template<typename NumericType>
struct GenericIndex
{
  NumericType x, y;
  explicit GenericIndex(const NumericType & x = 0, const NumericType & y = 0)
  : x(x), y(y)
  {
  }

  /**
   * @brief comparison operator that requires equal x and y
   */
  bool operator==(const GenericIndex & other) const
  {
    return x == other.x && y == other.y;
  }

  bool operator!=(const GenericIndex & other) const
  {
    return !operator==(other);
  }

  /**
   * @brief less than operator so object can be used in sets
   */
  bool operator<(const GenericIndex & other) const
  {
    return x < other.x || (x == other.x && y < other.y);
  }

  // Derived Comparators
  bool operator>(const GenericIndex & other) const {return other < *this;}
  bool operator<=(const GenericIndex & other) const {return !(*this > other);}
  bool operator>=(const GenericIndex & other) const {return !(*this < other);}

  /**
   * @brief String representation of this object
   */
  std::string toString() const
  {
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
  }
};

template<typename NumericType>
inline std::ostream & operator<<(std::ostream & stream, const GenericIndex<NumericType> & index)
{
  stream << index.toString();
  return stream;
}

using SignedIndex = GenericIndex<int>;
using Index = GenericIndex<unsigned int>;

}  // namespace mcr_nav_grid

#endif  // MCR_NAV_GRID__INDEX_HPP_
