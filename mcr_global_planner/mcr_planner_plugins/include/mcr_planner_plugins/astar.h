#ifndef _ASTAR_H_
#define _ASTAR_H_

#include "mcr_global_planner/potential_calculator.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <queue>
#include <vector>

namespace mcr_planner_plugins
{

class  AStar : public mcr_global_planner::PotentialCalculator
{
public:
  // Main PotentialCalculator interface
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, 
                const std::string &name,
                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
                mcr_global_planner::CostInterpreter::Ptr cost_interpreter) override;

  unsigned int updatePotentials(mcr_global_planner::PotentialGrid& potential_grid,
                                const geometry_msgs::msg::Pose2D& start, 
                                const geometry_msgs::msg::Pose2D& goal) override;
protected:
  /**
   * @brief Calculate the potential for index if not calculated already
   *
   * @param potential_grid Potential grid
   * @param prev_potential Potential of the previous cell
   * @param index Coordinates of cell to calculate
   * @param start_index Coordinates of start cell (for heuristic calculation)
   */
  void add(mcr_global_planner::PotentialGrid& potential_grid, double prev_potential,
           const mcr_nav_grid::Index& index, const mcr_nav_grid::Index& start_index);

  /**
   * @brief Calculate the heuristic value for a particular cell
   *
   * @param index Coordinates of cell to calculate
   * @param start_index Coordinates of start cell
   */
  float getHeuristicValue(const mcr_nav_grid::Index& index, const mcr_nav_grid::Index& start_index) const;

  /**
   * @brief Helper Class for sorting indexes by their heuristic
   */
  struct QueueEntry
  {
  public:
    QueueEntry(mcr_nav_grid::Index index, float heuristic) : i(index), cost(heuristic) {}
    mcr_nav_grid::Index i;
    float cost;
  };

  /**
   * @brief Comparator for sorting the QueueEntrys
   */
  struct QueueEntryComparator
  {
    bool operator()(const QueueEntry& a, const QueueEntry& b) const
    {
      return a.cost > b.cost;
    }
  };

  // Indexes sorted by heuristic
  using AStarQueue = std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryComparator>;
  AStarQueue queue_;
  bool manhattan_heuristic_;
  bool use_kernel_;
  double minimum_requeue_change_;
};




} // namespace mcr_planner_plugins


#endif