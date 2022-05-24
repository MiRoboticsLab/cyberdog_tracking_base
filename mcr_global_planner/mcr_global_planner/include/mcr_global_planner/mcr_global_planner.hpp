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

#ifndef MCR_GLOBAL_PLANNER__MCR_GLOBAL_PLANNER_HPP_
#define MCR_GLOBAL_PLANNER__MCR_GLOBAL_PLANNER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "mcr_global_planner/traceback.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "mcr_global_planner/potential_calculator.hpp"

namespace mcr_global_planner
{

class MCRGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  MCRGlobalPlanner();

  /**
   * @brief destructor
   */
  ~MCRGlobalPlanner();

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

  /**
   * @brief Check the costmap for any obstacles on this path
   * @param path Path to check
   * @return True if there are no obstacles
   */
  virtual bool isPlanValid(const nav_2d_msgs::msg::Path2D & path) const;

protected:
  // Subscription for parameter change
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  /**
   * @brief Callback executed when a paramter change is detected
   * @param event ParameterEvent message
   */
  void on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  // Path Caching Methods
  /**
   * @brief Check whether there is a valid cached path where the goal hasn't changed
   *
   * The precise goal and the grid coordinates of the goal are passed (redundantly) so the implementing method
   * can decide what precision to require for the goal to be the same. The default implementation uses the grid coords.
   *
   * Also sets the cached goal data member(s)
   *
   * @param local_goal Precise (floating point) goal
   * @param goal_x x coordinate of the goal in the grid
   * @param goal_y y coordinate of the goal in the grid
   * @return True if a path has been cached, the goal is the same as the cached paths, and the plan is valid
   */
  virtual bool hasValidCachedPath(
    const geometry_msgs::msg::Pose2D & local_goal,
    unsigned int goal_x, unsigned int goal_y);

  /**
   * @brief Whether the planner should always return a valid cached path without running the planning algorithm
   * @return True if the valid cached path should be returned without running the planning algorithm
   */
  virtual bool shouldReturnCachedPathImmediately() const;

  /**
   * @brief Given a cached path is available and a new path, should the new path be the one returned?
   * @param new_path The new path
   * @param new_path_cost The cost of the new path, according to the traceback
   * @return True if the new path should be the one returned. If False, return the cached one
   */
  virtual bool shouldReturnNewPath(
    const nav_2d_msgs::msg::Path2D & new_path,
    const double new_path_cost) const;

  geometry_msgs::msg::Pose2D transformPose(
    const geometry_msgs::msg::PoseStamped & in_pose,
    const std::string frame);

  // Plugins
  pluginlib::ClassLoader<PotentialCalculator> calc_loader_;
  PotentialCalculator::Ptr calculator_;
  pluginlib::ClassLoader<Traceback> traceback_loader_;
  Traceback::Ptr traceback_;

  // Key members
  PotentialGrid potential_grid_;
  CostInterpreter::Ptr cost_interpreter_;

  // Path Caching
  bool path_caching_;
  double improvement_threshold_;
  nav_2d_msgs::msg::Path2D cached_path_;
  unsigned int cached_goal_x_, cached_goal_y_;
  double cached_path_cost_;

  // potential publishing
  // nav_grid_pub_sub::ScaleGridPublisher<float> potential_pub_;

  // debug printing
  bool print_statistics_;

  nav2_util::LifecycleNode::SharedPtr node_;
  std::string global_frame_, name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
};
}  // namespace mcr_global_planner
#endif  // MCR_GLOBAL_PLANNER__MCR_GLOBAL_PLANNER_HPP_
