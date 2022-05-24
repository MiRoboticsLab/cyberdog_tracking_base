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
#include <string>
#include <memory>
#include <vector>
#include "mcr_global_planner/mcr_curve_planner.hpp"
#include "mcr_nav_grid/coordinate_conversion.hpp"
#include "mcr_global_planner/exceptions.hpp"
#include "nav2_util/node_utils.hpp"

namespace mcr_global_planner
{

void MCRCurvePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;       costmap_ros_ = costmap_ros;
  costmap_ = std::shared_ptr<nav2_costmap_2d::Costmap2D>(costmap_ros->getCostmap());
  global_frame_ = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(
    node_->get_logger(),
    "configuring plugin %s of type MCRCurvePlanner.", name_.c_str());

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".turning_radius", rclcpp::ParameterValue(
      0.3));
  node_->get_parameter(name_ + ".turning_radius", turning_radius_);
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".curve_type",
    rclcpp::ParameterValue("dubins"));
  node_->get_parameter(name_ + ".curve_type", curve_type_);
  if (curve_type_ != "dubins") {
    space_ = std::make_shared<ob::ReedsSheppStateSpace>(turning_radius_);
  } else {
    space_ = std::make_shared<ob::DubinsStateSpace>(turning_radius_);
  }
}


void MCRCurvePlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s of type MCRCurvePlanner", name_.c_str());
}

void MCRCurvePlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type MCRCurvePlanner", name_.c_str());
}

void MCRCurvePlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type MCRCurvePlanner", name_.c_str());
}


bool MCRCurvePlanner::isPlanValid(const nav_msgs::msg::Path & path) const
{
  // NB: Only checks for obstacles at each pose
  unsigned int x, y;
  const mcr_nav_grid::NavGridInfo & info = infoFromCostmap(costmap_ros_);
  for (geometry_msgs::msg::PoseStamped pose : path.poses) {
    if (!mcr_nav_grid::worldToGridBounded(info, pose.pose.position.x, pose.pose.position.y, x, y) ||
      costmap_->getCost(x, y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      return false;
    }
  }
  return true;
}

nav_msgs::msg::Path MCRCurvePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  ob::ScopedState<ob::ReedsSheppStateSpace> from(space_), to(space_), s(space_);
  std::vector<double> reals;

  from[0] = start.pose.position.x;
  from[1] = start.pose.position.y;
  double start_yaw = tf2::getYaw(start.pose.orientation);
  from[2] = start_yaw;

  to[0] = goal.pose.position.x;
  to[1] = goal.pose.position.y;
  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  to[2] = goal_yaw;

  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.header;
  size_t num_pts = space_->distance(from(), to()) / costmap_->getResolution();

  for (unsigned int i = 0; i <= num_pts; ++i) {
    space_->interpolate(from(), to(), static_cast<double>(i) / num_pts, s());
    reals = s.reals();
    pose.pose.position.x = reals[0];
    pose.pose.position.y = reals[1];

    tf2::Quaternion q;
    q.setRPY(0, 0, reals[2]);
    pose.pose.orientation = tf2::toMsg(q);

    path.poses.push_back(pose);
  }


  if (!isPlanValid(path)) {
    std::stringstream ss;
    ss << "No path is calculate from"
      " [" << start.pose.position.x << ", " << start.pose.position.y << ", " << start_yaw << "] "
      "to [" << goal.pose.position.x << ", " << goal.pose.position.y << ", " << goal_yaw << "]"
      "by forwardx_curve_planner: " << curve_type_;

    throw mcr_global_planner::NoGlobalPathException(ss.str());
  }

  return path;
}

}  // namespace mcr_global_planner
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mcr_global_planner::MCRCurvePlanner, nav2_core::GlobalPlanner)
