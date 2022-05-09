#ifndef _KERNEL_FUNCTION_H_
#define _KERNEL_FUNCTION_H_

#include "mcr_global_planner/potential.h"

namespace mcr_global_planner
{
/*
  Flag Mask Values such that NORTH + WEST = NORTHWEST
*/
  enum class CardinalDirection
  {
    STATIONARY = 0, // 0000
    NORTH = 1,     // 0001
    SOUTH = 2,     // 0010
    WEST = 4,      // 0100
    NORTHWEST = 5, // 0101
    SOUTHWEST = 6, // 0110
    EAST = 8,      // 1000
    NORTHEAST = 9, // 1001
    SOUTHEAST = 10 // 1010
  };

  inline CardinalDirection operator + (CardinalDirection a, CardinalDirection b)
  {
    return static_cast < CardinalDirection > (static_cast < int > (a) + static_cast < int > (b));
  }

  inline bool operator & (CardinalDirection a, CardinalDirection b)
  {
    return (static_cast < int > (a) & static_cast < int > (b)) > 0;
  }


/**
 * @brief potential calculation that uses multiple values of the neighboring cells
 *
 * The new potential is returned directly. Information about which cells were used in the computation
 * can be returned using the upstream variable.
 * For instance, if just the cell (x+1, y) was used in the calculation, upstream=EAST
 * If (x+1, y) and (x, y+1) were used, upstream=SOUTHEAST.
 *
 * @param potential_grid[in] potential grid where neighboring values are read from
 * @param cost[in] The cost to traversing to this cell
 * @param x[in] The x coordinate of this cell we're calculating potential for
 * @param y[in] The y coordinate
 * @param upstream[out] Direction of cells used in computation (if not null initially)
 * @return potential for this cell
 */
  static float calculateKernel(
    const PotentialGrid & potential_grid, unsigned char cost, unsigned int x, unsigned int y,
    CardinalDirection * upstream = nullptr)
  {
    // See README.md for more about this calculation
    // get neighboring potential values
    float south_p = y > 0 ? potential_grid(x, y - 1) : HIGH_POTENTIAL;
    float north_p = y < potential_grid.getHeight() - 1 ? potential_grid(x, y + 1) : HIGH_POTENTIAL;
    float west_p = x > 0 ? potential_grid(x - 1, y) : HIGH_POTENTIAL;
    float east_p = x < potential_grid.getWidth() - 1 ? potential_grid(x + 1, y) : HIGH_POTENTIAL;

    // Find the lowest neighbor on each axis
    // pa = min(P(A), P(B)) (aka north_p and south_p)
    // pc = min(P(C), P(D)) (aka west_p and east_p)
    float pa, pc;
    CardinalDirection xdir, ydir;
    if (north_p < south_p) {
      pa = north_p;
      ydir = CardinalDirection::NORTH;
    } else {
      pa = south_p;
      ydir = CardinalDirection::SOUTH;
    }

    if (west_p < east_p) {
      pc = west_p;
      xdir = CardinalDirection::WEST;
    } else {
      pc = east_p;
      xdir = CardinalDirection::EAST;
    }

    // cost was originally notated hf, h in the README
    float dc = pc - pa; // relative cost between pa, pc
    CardinalDirection mindir;

    // In our calculation, we assume P(A) <= P(C), so we flip as needed
    if (pa == HIGH_POTENTIAL || dc < 0) {      // pc is lowest
      dc = -dc;
      pa = pc;
      mindir = xdir;
    } else {
      mindir = ydir;
    }

    // If pa is lower and STILL infinite, then we can't calculate a good potential here
    if (std::isinf(pa)) {
      if (upstream) {
        *upstream = CardinalDirection::STATIONARY;
      }
      return pa;
    }

    // calculate new potential
    if (dc >= cost) {
      // if the difference is too large, use the "straightforward" calculation
      if (upstream) {
        *upstream = mindir;
      }
      return pa + cost;
    } else {
      // Otherwise, interpolate from both neighbors
      float dx = dc / cost;
      float v = -0.2301 * dx * dx + 0.5307 * dx + 0.7040;
      if (upstream) {
        *upstream = xdir + ydir;
      }
      return pa + cost * v;
    }
  }

} // namespace mcr_global_planner


#endif
