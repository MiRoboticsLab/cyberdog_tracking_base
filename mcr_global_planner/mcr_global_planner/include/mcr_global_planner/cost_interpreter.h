#ifndef _COST_INTERPRETER_H_
#define _COST_INTERPRETER_H_

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>
#include <memory>

namespace mcr_global_planner
{

  enum struct UnknownInterpretation { LETHAL, EXPENSIVE, FREE };
  const unsigned char LETHAL_COST = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  const float LETHAL_COST_F = static_cast < float > (LETHAL_COST);

  class CostInterpreter
  {
public:
    void initialize(
      rclcpp_lifecycle::LifecycleNode::SharedPtr parent_,
      const std::string & name,
      std::shared_ptr < nav2_costmap_2d::Costmap2DROS > costmap);

    void setConfiguration(
      const unsigned char neutral_cost,
      const float scale, const UnknownInterpretation mode);

    inline unsigned char getNeutralCost() const {return neutral_cost_;}

    inline float interpretCost(const unsigned char cost) const
    {
      return cached_costs_[cost];
    }

    inline float getCost(const unsigned int x, const unsigned int y) const
    {
      return interpretCost(costmap_->getCost(x, y));
    }

    inline bool isLethal(const float cost) const
    {
      return cost >= LETHAL_COST_F;
    }

    typedef std::shared_ptr < CostInterpreter > Ptr;

protected:
    std::string name_;
    void calculateCache();
    std::array < float, 256 > cached_costs_;
    unsigned char neutral_cost_;
    std::shared_ptr < nav2_costmap_2d::Costmap2D > costmap_;
  };

} // namespace mcr_global_planner


#endif
