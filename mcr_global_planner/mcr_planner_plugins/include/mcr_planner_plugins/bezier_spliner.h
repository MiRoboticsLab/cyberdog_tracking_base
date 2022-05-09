#ifndef _BEZIER_SPLINER_H_
#define _BEZIER_SPLINER_H_

#include <rclcpp/rclcpp.hpp>
#include <mcr_global_planner/spliner.h>

namespace mcr_planner_plugins
{

  class BezierSpliner: public mcr_global_planner::Spliner
  {
public:
//   BezierSpliner() {}

    virtual nav_msgs::msg::Path spline(
      const std::vector < geometry_msgs::msg::PoseStamped > & poses)
    override;
    void initialize(
      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
      const std::string & name,
      std::shared_ptr < nav2_costmap_2d::Costmap2DROS > costmap
    ) override;

protected:
    std::vector < geometry_msgs::msg::PoseStamped > && bezier(
      const std::vector < geometry_msgs::msg::PoseStamped > &control_points);

private:
    double det_ = 0.05;
    std::vector < std::vector < double >> cheat_sheet_;
    nav2_util::LifecycleNode::SharedPtr node_;
    std::string name_;
    std::shared_ptr < nav2_costmap_2d::Costmap2DROS > costmap_ros_;
  };
}

#endif
