#ifndef _POTENTIAL_H_
#define _POTENTIAL_H_

#include <limits>
#include "mcr_nav_grid/vector_nav_grid.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace mcr_global_planner {
/**
 * Default value for potential indicating it has not yet been calculated
 */
  const float HIGH_POTENTIAL = std::numeric_limits < float > ::max();

  typedef mcr_nav_grid::VectorNavGrid < float > PotentialGrid;


  static inline mcr_nav_grid::NavGridInfo infoFromCostmap(
    std::shared_ptr < nav2_costmap_2d::Costmap2DROS > costmap_ros)
  {
    mcr_nav_grid::NavGridInfo info;
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros->getCostmap();
    info.width = costmap->getSizeInCellsX();
    info.height = costmap->getSizeInCellsY();
    info.resolution = costmap->getResolution();
    info.frame_id = costmap_ros->getGlobalFrameID();
    info.origin_x = costmap->getOriginX();
    info.origin_y = costmap->getOriginY();
    return info;
  }

}
#endif
