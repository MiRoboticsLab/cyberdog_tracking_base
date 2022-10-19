// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights
// reserved.
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

#include "mcr_global_planner/mcr_spline_planner.hpp"

#include <memory>
#include <string>
#include <vector>

#include "mcr_global_planner/exceptions.hpp"
#include "mcr_global_planner/potential.hpp"
#include "mcr_global_planner/spliner.hpp"
#include "mcr_nav_grid/coordinate_conversion.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::literals::chrono_literals;

namespace mcr_global_planner {

MCRSplinePlanner::MCRSplinePlanner()
    : tf_(nullptr),
      costmap_(nullptr),
      spliner_loader_("mcr_global_planner", "mcr_global_planner::Spliner") {}

void MCRSplinePlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ =
      std::shared_ptr<nav2_costmap_2d::Costmap2D>(costmap_ros->getCostmap());
  global_frame_ = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(node_->get_logger(),
              "configuring plugin %s of type MCRSplinePlanner.", name_.c_str());

  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".spline_name", rclcpp::ParameterValue("bezier"));
  node_->get_parameter(name_ + ".spline_name", spliner_name_);

  std::string plugin;
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + "." + spliner_name_ + ".plugin",
      rclcpp::ParameterValue("mcr_planner_plugins::BezierSpliner"));
  node_->get_parameter(name_ + "." + spliner_name_ + ".plugin", plugin);

  RCLCPP_INFO(node_->get_logger(), "MCRSplinePlanner Using Spline Type \"%s\"",
              spliner_name_.c_str());
  spliner_ = spliner_loader_.createUniqueInstance(plugin);
  spliner_->initialize(node_, name_, costmap_ros_);
}

void MCRSplinePlanner::cleanup() {
  RCLCPP_INFO(node_->get_logger(),
              "Cleaning up plugin %s of type MCRSplinePlanner", name_.c_str());
}

void MCRSplinePlanner::activate() {
  RCLCPP_INFO(node_->get_logger(),
              "Activating plugin %s of type MCRSplinePlanner", name_.c_str());
}

void MCRSplinePlanner::deactivate() {
  RCLCPP_INFO(node_->get_logger(),
              "Deactivating plugin %s of type MCRSplinePlanner", name_.c_str());
}

bool MCRSplinePlanner::isPlanValid(const nav_msgs::msg::Path& path) const {
  // NB: Only checks for obstacles at each pose
  unsigned int x, y;
  const mcr_nav_grid::NavGridInfo& info = infoFromCostmap(costmap_ros_);
  for (geometry_msgs::msg::PoseStamped pose : path.poses) {
    if (!mcr_nav_grid::worldToGridBounded(info, pose.pose.position.x,
                                          pose.pose.position.y, x, y)){
      continue;
    }

    if(costmap_->getCost(x, y) >=
       nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }
  }
  return true;
}

nav_msgs::msg::Path MCRSplinePlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  // path.poses.push_back(goal);

  path.poses.push_back(start);
  geometry_msgs::msg::Point s = start.pose.position;

  geometry_msgs::msg::Point e = goal.pose.position;
  double sq_dist = (e.x - s.x) * (e.x - s.x) + (e.y - s.y) * (e.y - s.y);

  double resolution = costmap_->getResolution();
  double sq_resolution = resolution * resolution;

  if (sq_dist > sq_resolution) {
    // add points in-between
    double diff = sqrt(sq_dist) - resolution;
    double steps_double = ceil(diff / resolution) + 1.0;
    int steps = static_cast<int>(steps_double);

    double delta_x = (e.x - s.x) / steps_double;
    double delta_y = (e.y - s.y) / steps_double;

    for (int j = 1; j < steps; ++j) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = s.x + j * delta_x;
      pose.pose.position.y = s.y + j * delta_y;
      path.poses.push_back(pose);
    }
  }
  path.poses.push_back(goal);
  path.header.frame_id = global_frame_;

  if (!isPlanValid(path)) {
    std::stringstream ss;
    ss << "No valid path is calculated from"
      " [" << start.pose.position.x << ", " << start.pose.position.y << "] "
      "to [" << goal.pose.position.x << ", " << goal.pose.position.y << "]"
      "by mcr_spline_planner: " << spliner_name_;

    throw mcr_global_planner::NoGlobalPathException(ss.str());
  }

  return path;
}

nav_msgs::msg::Path MCRSplinePlanner::createPlan(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
  if (poses.size() < 2) {
    RCLCPP_INFO(node_->get_logger(), "invalid poses number : %lu, less than 2.",
                poses.size());
    return nav_msgs::msg::Path();
  } else if (poses.size() == 2) {
    return createPlan(poses[0], poses[1]);
  } else {
    nav_msgs::msg::Path path = spliner_->spline(poses);
    path.header.frame_id = global_frame_;

    if (!isPlanValid(path)) {
      std::stringstream ss;
      ss << "No valid path is calculated from " << poses.size() << " poses "
        "by mcr_spline_planner: " << spliner_name_;
      throw mcr_global_planner::NoGlobalPathException(ss.str());
    }
    return path;
  }
}

}  // namespace mcr_global_planner
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mcr_global_planner::MCRSplinePlanner,
                       nav2_core::GlobalPlanner)
