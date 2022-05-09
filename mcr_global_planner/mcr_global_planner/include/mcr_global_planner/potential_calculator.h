#ifndef _POTENTIAL_CALCULATOR_H_
#define _POTENTIAL_CALCULATOR_H_

#include "mcr_global_planner/cost_interpreter.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "mcr_global_planner/potential.h"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace mcr_global_planner {

  class PotentialCalculator {
public:
    using Ptr = std::shared_ptr < PotentialCalculator >;

    PotentialCalculator() : cost_interpreter_(nullptr) {
    }
    virtual ~PotentialCalculator() = default;
    /**
     * @brief Initialize function
     *
     * Can be overridden to grab parameters or direct access to the costmap
     *
     * @param private_nh NodeHandle to read parameters from
     * @param costmap Pointer to costmap (possibly for tracking changes)
     * @param cost_interpreter CostInterpreter pointer
     */
    virtual void initialize(
      rclcpp_lifecycle::LifecycleNode::SharedPtr /*node*/,
      const std::string & name,
      std::shared_ptr < nav2_costmap_2d::Costmap2DROS > /*costmap*/,
      CostInterpreter::Ptr cost_interpreter)
    {cost_interpreter_ = cost_interpreter; name_ = name;}

    /**
     * @brief Main potential calculation function
     *
     * @param potential_grid potentials are written into here
     * @param start Start pose, in the same frame as the potential
     * @param goal Goal pose, in the same frame as the potential
     * @return Number of cells expanded. Used for comparing expansion strategies.
     */
    virtual unsigned int updatePotentials(
      PotentialGrid & potential_grid,
      const geometry_msgs::msg::Pose2D & start,
      const geometry_msgs::msg::Pose2D & goal) = 0;

protected:
    CostInterpreter::Ptr cost_interpreter_;
    std::string name_;
  };


}

#endif
