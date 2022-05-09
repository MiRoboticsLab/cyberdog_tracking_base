
#include <mcr_planner_plugins/gradient_path.h>
#include <mcr_nav_grid/coordinate_conversion.h>
#include <mcr_global_planner/exceptions.h>
#include <nav2_util/node_utils.hpp>
#include <math.h>
#include <string>


using mcr_global_planner::HIGH_POTENTIAL;

namespace mcr_planner_plugins
{
void GradientPath::initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, 
                              const std::string &name,
                              mcr_global_planner::CostInterpreter::Ptr cost_interpreter)
{
  name_ = name;
  cost_interpreter_ = cost_interpreter;
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".step_size", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".step_size", step_size_);
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".lethal_cost", rclcpp::ParameterValue(250.0));
  node->get_parameter(name_ + ".lethal_cost", lethal_cost_);
  
  // As a safeguard against infinitely iterating, we limit the number of steps we take, 
  // relative to the size of the costmap/potential grid. The maximum number of iterations
  // is width * height * iteration_factor.
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".iteration_factor", rclcpp::ParameterValue(4.0));
  node->get_parameter(name_ + ".iteration_factor", iteration_factor_);

  // NavFn would not attempt to calculate a gradient if one of the neighboring cells had not been initialized.
  // Setting this to true will replicate that behavior
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".grid_step_near_high", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".grid_step_near_high", grid_step_near_high_);
}

nav_2d_msgs::msg::Path2D GradientPath::getPath(const mcr_global_planner::PotentialGrid& potential_grid,
                                          const geometry_msgs::msg::Pose2D& start, 
                                          const geometry_msgs::msg::Pose2D& goal,
                                          double& path_cost)
{
  const mcr_nav_grid::NavGridInfo& info = potential_grid.getInfo();
  if (gradx_.getInfo() != info)
  {
    gradx_.setInfo(info);
    grady_.setInfo(info);
  }
  gradx_.reset();
  grady_.reset();

  nav_2d_msgs::msg::Path2D path;
  path_cost = 0.0;

  double map_x, map_y;
  worldToGrid(info, start.x, start.y, map_x, map_y);
  // Running Variables
  //   index is the current index
  //   dx and dy represent the precise floating point coordinates within the cell [0, 1)
  double x_int_part, y_int_part;
  float dx = modf(map_x, &x_int_part),
        dy = modf(map_y, &y_int_part);
  mcr_nav_grid::Index index(static_cast<unsigned int>(x_int_part), static_cast<unsigned int>(y_int_part));

  // Variables to check for loop completion
  worldToGrid(info, goal.x, goal.y, map_x, map_y);
  double finish_x = floor(map_x),
         finish_y = floor(map_y);
  mcr_nav_grid::Index finish_index(static_cast<unsigned int>(finish_x), static_cast<unsigned int>(finish_y));

  unsigned int c = 0, max_iterations = static_cast<unsigned int>(potential_grid.size() * iteration_factor_);
  geometry_msgs::msg::Pose2D current;

  while (c++ < max_iterations)
  {
    current.x = index.x + dx;
    current.y = index.y + dy;
    path.poses.push_back(current);
    path_cost += step_size_ * cost_interpreter_->getCost(index.x, index.y);

    // check if near goal
    if (index == finish_index || 
      (fabs(current.x - finish_x) <= step_size_ && fabs(current.y - finish_y) <= step_size_))
    {
      nav_2d_msgs::msg::Path2D world_path = mapPathToWorldPath(path, potential_grid.getInfo());
      world_path.poses.push_back(goal);
      return world_path;
    }

    // Check if the most recent pose, and two before the most recent pose are the same
    bool oscillation_detected = false;
    int npath = path.poses.size();
    if (npath > 2 && path.poses[npath - 1].x == path.poses[npath - 3].x
                  && path.poses[npath - 1].y == path.poses[npath - 3].y)
    {
      oscillation_detected = true;
    }

    /* We need to examine eight different neighboring cells.
     * +----------+----------+----------+
     * | index_nw | index_n  | index_ne |
     * +----------+----------+----------+
     * | index_w  | index    | index_e  |
     * +----------+----------+----------+
     * | index_sw | index_s  | index_se |
     * +----------+----------+----------+
     *
     * The original implementors of navfn unrolled the loops for interating over all the cells.
     * We copy their technique here.
     */
    if (shouldGridStep(potential_grid, index) || oscillation_detected)
    {
      index = gridStep(potential_grid, index);
      dx = 0.0;
      dy = 0.0;
    }else{// have a good gradient here
      /* Calculate the gradient of four cells
       * +----------+----------+----------+
       * |          | index_n  | index_ne |
       * +----------o----------o----------+
       * |          | index    | index_e  |
       * +----------o----------o----------+
       * |          |          |          |
       * +----------+----------+----------+
       * Our position is somewhere in the index cell, relative to dx and dy
       * The gradient we calculate is relative to the lower left corner of each cell (as marked by 'o's above)
       * Thus, the precise graident we will follow is two-dimensional linear combination of four gradients
       * surrounding our current cell.
       */
      mcr_nav_grid::Index  index_e(index.x + 1, index.y),
                       index_n(index.x,     index.y + 1),
                      index_ne(index.x + 1, index.y + 1);
      calculateGradient(potential_grid, index);
      calculateGradient(potential_grid, index_e);
      calculateGradient(potential_grid, index_n);
      calculateGradient(potential_grid, index_ne);

      // get interpolated gradient
      float x1 = (1.0 - dx) * gradx_(index) + dx * gradx_(index_e);
      float x2 = (1.0 - dx) * gradx_(index_n) + dx * gradx_(index_ne);
      float x = (1.0 - dy) * x1 + dy * x2;  // interpolated x
      float y1 = (1.0 - dx) * grady_(index) + dx * grady_(index_e);
      float y2 = (1.0 - dx) * grady_(index_n) + dx * grady_(index_ne);
      float y = (1.0 - dy) * y1 + dy * y2;  // interpolated y

      // check for zero gradient
      if (x == 0.0 && y == 0.0)
      {
        // This generally shouldn't happen, but can if using a not well-formed potential function
        // If there's no gradient, move along the grid.
        index = gridStep(potential_grid, index);
        dx = 0.0;
        dy = 0.0;
        continue;
      }

      // move in the right direction
      float ss = step_size_ / hypot(x, y);
      dx += x * ss;
      dy += y * ss;

      // check if we have moved out of the current cell
      if (dx >= 1.0)
      {
        index.x++;
        dx -= 1.0;
      }
      if (dx < 0.0)
      {
        index.x--;
        dx += 1.0;
      }
      if (dy >= 1.0)
      {
        index.y++;
        dy -= 1.0;
      }
      if (dy < 0.0)
      {
        index.y--;
        dy += 1.0;
      }
    }

  }

  return path;
}

bool GradientPath::shouldGridStep(const mcr_global_planner::PotentialGrid& potential_grid,
                                  const mcr_nav_grid::Index& index)
{
  bool near_edge = index.x == 0 || index.x >= potential_grid.getWidth() - 1 ||
                   index.y == 0 || index.y >= potential_grid.getHeight() - 1;
  if (near_edge || !grid_step_near_high_)
    return near_edge;

  // check for potentials at eight positions near cell
  return potential_grid(index) >= HIGH_POTENTIAL ||
      potential_grid(index.x + 1, index.y)     >= HIGH_POTENTIAL ||
      potential_grid(index.x - 1, index.y)     >= HIGH_POTENTIAL ||
      potential_grid(index.x,     index.y + 1) >= HIGH_POTENTIAL ||
      potential_grid(index.x,     index.y - 1) >= HIGH_POTENTIAL ||
      potential_grid(index.x + 1, index.y + 1) >= HIGH_POTENTIAL ||
      potential_grid(index.x + 1, index.y - 1) >= HIGH_POTENTIAL ||
      potential_grid(index.x - 1, index.y + 1) >= HIGH_POTENTIAL ||
      potential_grid(index.x - 1, index.y - 1) >= HIGH_POTENTIAL;
}

mcr_nav_grid::Index GradientPath::gridStep(const mcr_global_planner::PotentialGrid& potential_grid,
                                       const mcr_nav_grid::Index& index)
{
  // check eight neighbors to find the lowest
  mcr_nav_grid::Index min_index = index;
  float min_potential = potential_grid(index);

  // Unrolled loop

  // Check the south
  if (index.y > 0)
  {
    mcr_nav_grid::Index index_s(index.x, index.y - 1);
    if (potential_grid(index_s) < min_potential)
    {
      min_potential = potential_grid(index_s);
      min_index = index_s;
    }

    // check the south west
    if (index.x > 0)
    {
      mcr_nav_grid::Index index_sw(index.x - 1, index.y - 1);
      if (potential_grid(index_sw) < min_potential)
      {
        min_potential = potential_grid(index_sw);
        min_index = index_sw;
      }
    }

    // check the south east
    if (index.x < potential_grid.getWidth() - 1)
    {
      mcr_nav_grid::Index index_se(index.x + 1, index.y - 1);
      if (potential_grid(index_se) < min_potential)
      {
        min_potential = potential_grid(index_se);
        min_index = index_se;
      }
    }
  }

  // check the west
  if (index.x > 0)
  {
    mcr_nav_grid::Index index_w(index.x - 1, index.y);
    if (potential_grid(index_w) < min_potential)
    {
      min_potential = potential_grid(index_w);
      min_index = index_w;
    }
  }
  // check the east
  if (index.x < potential_grid.getWidth() - 1)
  {
    mcr_nav_grid::Index index_e(index.x + 1, index.y);
    if (potential_grid(index_e) < min_potential)
    {
      min_potential = potential_grid(index_e);
      min_index = index_e;
    }
  }

  // Check the north
  if (index.y < potential_grid.getHeight() - 1)
  {
    mcr_nav_grid::Index index_n(index.x, index.y + 1);
    if (potential_grid(index_n) < min_potential)
    {
      min_potential = potential_grid(index_n);
      min_index = index_n;
    }

    // check the north west
    if (index.x > 0)
    {
      mcr_nav_grid::Index index_nw(index.x - 1, index.y + 1);
      if (potential_grid(index_nw) < min_potential)
      {
        min_potential = potential_grid(index_nw);
        min_index = index_nw;
      }
    }

    // check the north east
    if (index.x < potential_grid.getWidth() - 1)
    {
      mcr_nav_grid::Index index_ne(index.x + 1, index.y + 1);
      if (potential_grid(index_ne) < min_potential)
      {
        min_potential = potential_grid(index_ne);
        min_index = index_ne;
      }
    }
  }
  if (min_index == index)
  {
    throw mcr_global_planner::NoGlobalPathException("No path found. Local min.");
  }
  else if (potential_grid(min_index) >= HIGH_POTENTIAL)
  {
    throw mcr_global_planner::NoGlobalPathException("No path found, high potential");
  }
  return min_index;
}

void GradientPath::calculateGradient(const mcr_global_planner::PotentialGrid& potential_grid,
                                     const mcr_nav_grid::Index& index)
{
  // If non-zero, gradient already calculated, skip
  if (gradx_(index) + grady_(index) > 0.0)
    return;

  float cv = potential_grid(index);
  float dx = 0.0;
  float dy = 0.0;

  float south_p = index.y > 0                              ? potential_grid(index.x, index.y - 1) : HIGH_POTENTIAL;
  float north_p = index.y < potential_grid.getHeight() - 1 ? potential_grid(index.x, index.y + 1) : HIGH_POTENTIAL;
  float west_p =  index.x > 0                              ? potential_grid(index.x - 1, index.y) : HIGH_POTENTIAL;
  float east_p =  index.x < potential_grid.getWidth() - 1  ? potential_grid(index.x + 1, index.y) : HIGH_POTENTIAL;

  // check if in an obstacle
  if (cv >= HIGH_POTENTIAL)
  {
    if (west_p < HIGH_POTENTIAL)
      dx = -lethal_cost_;
    else if (east_p < HIGH_POTENTIAL)
      dx = lethal_cost_;

    if (south_p < HIGH_POTENTIAL)
      dy = -lethal_cost_;
    else if (north_p < HIGH_POTENTIAL)
      dy = lethal_cost_;
  }
  else  // not in an obstacle
  {
    // dx calc, average to sides
    if (west_p < HIGH_POTENTIAL)
      dx += west_p - cv;
    if (east_p < HIGH_POTENTIAL)
      dx += cv - east_p;

    // dy calc, average to sides
    if (south_p < HIGH_POTENTIAL)
      dy += south_p - cv;
    if (north_p < HIGH_POTENTIAL)
      dy += cv - north_p;
  }

  // normalize
  float norm = hypot(dx, dy);
  if (norm > 0)
  {
    norm = 1.0 / norm;
    gradx_.setValue(index, norm * dx);
    grady_.setValue(index, norm * dy);
  }
}

}  // namespace mcr_planner_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mcr_planner_plugins::GradientPath, mcr_global_planner::Traceback)
