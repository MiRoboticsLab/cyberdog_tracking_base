#ifndef _GRID_PATH_H_
#define _GRID_PATH_H_

#include "mcr_global_planner/traceback.h"

namespace mcr_planner_plugins
{

class GridPath : public mcr_global_planner::Traceback
{
public:
  // Main Traceback interface
  nav_2d_msgs::msg::Path2D getPath(const mcr_global_planner::PotentialGrid& potential_grid,
                              const geometry_msgs::msg::Pose2D& start,
                              const geometry_msgs::msg::Pose2D& goal,
                              double& path_cost) override;
};    
} // namespace mcr_planner_plugins


#endif