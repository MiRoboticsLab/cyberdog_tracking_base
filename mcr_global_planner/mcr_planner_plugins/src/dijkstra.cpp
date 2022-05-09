#include <mcr_planner_plugins/dijkstra.h>
#include <mcr_nav_grid/coordinate_conversion.h>
#include <mcr_global_planner/exceptions.h>
#include <mcr_global_planner/kernel_function.h>
#include <queue>

namespace mcr_planner_plugins
{
unsigned int Dijkstra::updatePotentials(
  mcr_global_planner::PotentialGrid & potential_grid,
  const geometry_msgs::msg::Pose2D & start,
  const geometry_msgs::msg::Pose2D & goal)
{
  const mcr_nav_grid::NavGridInfo & info = potential_grid.getInfo();
  queue_ = std::queue<mcr_nav_grid::Index>();
  potential_grid.reset();

  mcr_nav_grid::Index goal_i;
  worldToGridBounded(info, goal.x, goal.y, goal_i.x, goal_i.y);
  queue_.push(goal_i);
  potential_grid.setValue(goal_i, 0.0);

  mcr_nav_grid::Index start_i;
  worldToGridBounded(info, start.x, start.y, start_i.x, start_i.y);
  unsigned int c = 0;

  while (!queue_.empty()) {
    mcr_nav_grid::Index i = queue_.front();
    queue_.pop();
    c++;

    if (i == start_i) {return c;}

    if (i.x > 0) {
      add(potential_grid, mcr_nav_grid::Index(i.x - 1, i.y));
    }
    if (i.y > 0) {
      add(potential_grid, mcr_nav_grid::Index(i.x, i.y - 1));
    }

    if (i.x < info.width - 1) {
      add(potential_grid, mcr_nav_grid::Index(i.x + 1, i.y));
    }
    if (i.y < info.height - 1) {
      add(potential_grid, mcr_nav_grid::Index(i.x, i.y + 1));
    }
  }

  throw mcr_global_planner::NoGlobalPathException();
}

void Dijkstra::add(
  mcr_global_planner::PotentialGrid & potential_grid,
  mcr_nav_grid::Index next_index)
{
  if (potential_grid(next_index.x, next_index.y) < mcr_global_planner::HIGH_POTENTIAL) {
    return;
  }

  float cost = cost_interpreter_->getCost(next_index.x, next_index.y);
  if (cost_interpreter_->isLethal(cost)) {
    return;
  }
  potential_grid.setValue(
    next_index,
    mcr_global_planner::calculateKernel(potential_grid, cost, next_index.x, next_index.y));
  queue_.push(next_index);
}

}  // namespace mcr_planner_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mcr_planner_plugins::Dijkstra, mcr_global_planner::PotentialCalculator)
