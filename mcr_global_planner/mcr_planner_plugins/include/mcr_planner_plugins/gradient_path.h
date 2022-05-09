#ifndef _GRADIENT_PATH_H_
#define _GRADIENT_PATH_H_


#include <mcr_global_planner/traceback.h>
#include <mcr_nav_grid/vector_nav_grid.h>

namespace mcr_planner_plugins
{
/**
 * @class GradientPath
 * @brief Traceback function that creates a smooth gradient from the start to the goal
 */
  class GradientPath: public mcr_global_planner::Traceback
  {
public:
    // Main Traceback interface
    void initialize(
      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
      const std::string & name,
      mcr_global_planner::CostInterpreter::Ptr cost_interpreter) override;

    nav_2d_msgs::msg::Path2D getPath(
      const mcr_global_planner::PotentialGrid & potential_grid,
      const geometry_msgs::msg::Pose2D & start,
      const geometry_msgs::msg::Pose2D & goal,
      double & path_cost) override;

protected:
    bool shouldGridStep(
      const mcr_global_planner::PotentialGrid & potential_grid,
      const mcr_nav_grid::Index & index);

    mcr_nav_grid::Index gridStep(
      const mcr_global_planner::PotentialGrid & potential_grid,
      const mcr_nav_grid::Index & index);

    inline void calculateGradient(
      const mcr_global_planner::PotentialGrid & potential_grid,
      const mcr_nav_grid::Index & index);

    double step_size_;
    double lethal_cost_;
    double iteration_factor_;
    bool grid_step_near_high_;
    mcr_nav_grid::VectorNavGrid < double > gradx_, grady_;
  };
}  // namespace mcr_planner_plugins

#endif
