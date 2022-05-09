#include <mcr_planner_plugins/grid_path.h>
#include <mcr_nav_grid/coordinate_conversion.h>
#include <mcr_global_planner/exceptions.h>
#include <limits>

namespace mcr_planner_plugins
{

nav_2d_msgs::msg::Path2D GridPath::getPath(
  const mcr_global_planner::PotentialGrid & potential_grid,
  const geometry_msgs::msg::Pose2D & start,
  const geometry_msgs::msg::Pose2D & goal,
  double & path_cost)
{
  const mcr_nav_grid::NavGridInfo & info = potential_grid.getInfo();
  nav_2d_msgs::msg::Path2D path;
  path_cost = 0.0;

  unsigned int current_x = 0, current_y = 0;
  worldToGridBounded(info, start.x, start.y, current_x, current_y);

  // Add 0.5 to represent center of the cell
  geometry_msgs::msg::Pose2D current;
  current.x = current_x + 0.5;
  current.y = current_y + 0.5;
  path.poses.push_back(current);

  unsigned int goal_index = potential_grid.getIndex(goal.x, goal.y);

  // Main Loop
  while (potential_grid.getIndex(current_x, current_y) != goal_index) {
    float min_val = std::numeric_limits<float>::max();
    unsigned int min_x = 0, min_y = 0;
    int distance_sq = 0;
    for (int xd = -1; xd <= 1; xd++) {
      if ((current_x == 0 && xd == -1) || (current_x == info.width - 1 && xd == 1)) {continue;}
      for (int yd = -1; yd <= 1; yd++) {
        if ((current_y == 0 && yd == -1) || (current_y == info.height - 1 && yd == 1)) {continue;}
        if (xd == 0 && yd == 0) {
          continue;
        }
        unsigned int x = current_x + xd, y = current_y + yd;
        int index = potential_grid.getIndex(x, y);
        if (potential_grid[index] < min_val) {
          min_val = potential_grid[index];
          min_x = x;
          min_y = y;
          distance_sq = abs(xd) + abs(yd);
        }
      }
    }
    if (distance_sq == 0) {
      throw mcr_global_planner::NoGlobalPathException("Reached dead end in traceback.");
    }

    double distance;
    if (distance_sq == 1) {
      distance = 0.5;
    } else {
      distance = M_SQRT1_2;  // sqrt(2)/2

    }
    path_cost += distance * cost_interpreter_->getCost(current_x, current_y);

    // Move to the Min Neighbor
    current_x = min_x;
    current_y = min_y;

    // Add 0.5 to represent center of the cell
    current.x = current_x + 0.5;
    current.y = current_y + 0.5;
    path.poses.push_back(current);
    path_cost += distance * cost_interpreter_->getCost(current_x, current_y);
  }
  return mapPathToWorldPath(path, info);
}

}  // namespace mcr_planner_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mcr_planner_plugins::GridPath, mcr_global_planner::Traceback)
