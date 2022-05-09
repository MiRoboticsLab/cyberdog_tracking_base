#ifndef _DIJKSTRA_H_
#define _DIJKSTRA_H_

#include <mcr_global_planner/potential_calculator.h>
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
  unsigned int updatePotentials(mcr_global_planner::PotentialGrid& potential_grid,
                                const geometry_msgs::msg::Pose2D& start, 
                                const geometry_msgs::msg::Pose2D& goal) override;
protected:
  /**
   * @brief Calculate the potential for next_index if not calculated already
   *
   * @param potential_grid Potential grid
   * @param next_index Coordinates of cell to calculate
   */
  void add(mcr_global_planner::PotentialGrid& potential_grid, mcr_nav_grid::Index next_index);

  std::queue<mcr_nav_grid::Index> queue_;
};
}
#endif