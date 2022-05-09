#include <mcr_planner_plugins/von_neumann_path.h>
#include <mcr_nav_grid/coordinate_conversion.h>
#include <mcr_global_planner/exceptions.h>
#include <limits>

namespace mcr_planner_plugins
{
nav_2d_msgs::msg::Path2D VonNeumannPath::getPath(
  const mcr_global_planner::PotentialGrid & potential_grid,
  const geometry_msgs::msg::Pose2D & start,
  const geometry_msgs::msg::Pose2D & goal,
  double & path_cost)
{
  nav_2d_msgs::msg::Path2D path;
  path_cost = 0.0;

  unsigned int current_x = 0, current_y = 0;
  worldToGridBounded(potential_grid.getInfo(), start.x, start.y, current_x, current_y);

  // Add 0.5 to represent center of the cell
  geometry_msgs::msg::Pose2D current;
  current.x = current_x + 0.5;
  current.y = current_y + 0.5;
  path.poses.push_back(current);
  path_cost += cost_interpreter_->getCost(current_x, current_y);

  unsigned int goal_index = potential_grid.getIndex(goal.x, goal.y);

  // Main Loop
  while (potential_grid.getIndex(current_x, current_y) != goal_index) {
    float min_val = std::numeric_limits<float>::max();
    unsigned int min_x = 0, min_y = 0;
    unsigned int x, y;
    int index;

    // Check the four neighbors
    if (current_x > 0) {
      x = current_x - 1;
      y = current_y;
      index = potential_grid.getIndex(x, y);
      if (potential_grid[index] < min_val) {
        min_val = potential_grid[index];
        min_x = x;
        min_y = y;
      }
    }
    if (current_x < potential_grid.getWidth() - 1) {
      x = current_x + 1;
      y = current_y;
      index = potential_grid.getIndex(x, y);
      if (potential_grid[index] < min_val) {
        min_val = potential_grid[index];
        min_x = x;
        min_y = y;
      }
    }
    if (current_y > 0) {
      x = current_x;
      y = current_y - 1;
      index = potential_grid.getIndex(x, y);
      if (potential_grid[index] < min_val) {
        min_val = potential_grid[index];
        min_x = x;
        min_y = y;
      }
    }
    if (current_y < potential_grid.getHeight() - 1) {
      x = current_x;
      y = current_y + 1;
      index = potential_grid.getIndex(x, y);
      if (potential_grid[index] < min_val) {
        min_val = potential_grid[index];
        min_x = x;
        min_y = y;
      }
    }

    if (min_val == std::numeric_limits<float>::max()) {
      throw mcr_global_planner::NoGlobalPathException("Reached dead end in traceback.");
    }

    // Move to the Min Neighbor
    current_x = min_x;
    current_y = min_y;

    // Add 0.5 to represent center of the cell
    current.x = current_x + 0.5;
    current.y = current_y + 0.5;
    path.poses.push_back(current);
    path_cost += cost_interpreter_->getCost(current_x, current_y);
  }
  return mapPathToWorldPath(path, potential_grid.getInfo());
}

}  // namespace mcr_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mcr_planner_plugins::VonNeumannPath, mcr_global_planner::Traceback)
