#ifndef _SPLINER_H_
#define _SPLINER_H_

#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <nav_msgs/msg/path.hpp>

namespace mcr_global_planner
{
  class Spliner
  {
public:
    using Ptr = std::shared_ptr < Spliner >;
    Spliner() {
    }
    virtual ~Spliner() = default;
    virtual nav_msgs::msg::Path spline(
      const std::vector < geometry_msgs::msg::PoseStamped > & poses) = 0;
    virtual void initialize(
      rclcpp_lifecycle::LifecycleNode::SharedPtr /*node*/,
      const std::string & name,
      std::shared_ptr < nav2_costmap_2d::Costmap2DROS >/*costmap*/
    ) = 0;

    std::string name_;
  };
}
#endif
