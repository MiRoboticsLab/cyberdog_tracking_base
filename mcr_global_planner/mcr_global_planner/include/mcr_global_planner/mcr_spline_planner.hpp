// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MCR_GLOBAL_PLANNER__MCR_SPLINE_PLANNER_HPP_
#define MCR_GLOBAL_PLANNER__MCR_SPLINE_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "mcr_global_planner/cost_interpreter.hpp"
#include "mcr_global_planner/potential_calculator.hpp"
#include "mcr_global_planner/spliner.hpp"
#include "mcr_global_planner/potential.hpp"
#include "mcr_global_planner/traceback.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mcr_global_planner
{
class Spliner;

class MCRSplinePlanner : public nav2_core::GlobalPlanner
{
public:
  MCRSplinePlanner();
  ~MCRSplinePlanner()
  {
  }

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;


  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
  nav_msgs::msg::Path createPlan(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses) override;

  virtual bool isPlanValid(const nav_msgs::msg::Path & path) const;

protected:
  nav2_util::LifecycleNode::SharedPtr node_;
  std::string global_frame_, name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;

  std::string spliner_name_;

  pluginlib::ClassLoader<Spliner> spliner_loader_;
  Traceback::Ptr traceback_;
  rclcpp::TimerBase::SharedPtr timer_;
  Spliner::Ptr spliner_;
};
}  // namespace mcr_global_planner

#endif  // MCR_GLOBAL_PLANNER__MCR_SPLINE_PLANNER_HPP_
