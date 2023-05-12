// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights
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

#include "nav2_util/node_utils.hpp"

using namespace std::literals::chrono_literals;

namespace mcr_global_planner {

MCRSplinePlanner::MCRSplinePlanner()
    : tf_(nullptr),
      costmap_(nullptr) {}

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

nav_msgs::msg::Path MCRSplinePlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
  nav_msgs::msg::Path path;
  path.header.frame_id = goal.header.frame_id;
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
      pose.header.frame_id = goal.header.frame_id;
      path.poses.push_back(pose);
    }
  }
  path.poses.push_back(goal);
  path.header.frame_id = global_frame_;

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
    nav_msgs::msg::Path path = spline(poses);
    path.header.frame_id = global_frame_;
    return path;
  }
}

nav_msgs::msg::Path MCRSplinePlanner::spline(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
  nav_msgs::msg::Path path;
  RCLCPP_INFO(node_->get_logger(), "pose size: %ld", poses.size());
  path.header.stamp = node_->now();
  if (poses.size() < 2) {
    return path;
  }

  std::vector<geometry_msgs::msg::PoseStamped> control_points;
  for (size_t i = 0; i < poses.size() - 1; i++) {
    control_points.clear();
    control_points.push_back(poses[i]);
    control_points.push_back(poses[i + 1]);
    std::vector<geometry_msgs::msg::PoseStamped>&& poses_t =
        interpolation(control_points);
    path.poses.insert(path.poses.end(), poses_t.begin(), poses_t.end());
  }
  return path;
}

std::vector<geometry_msgs::msg::PoseStamped>&&
MCRSplinePlanner::interpolation(
    const std::vector<geometry_msgs::msg::PoseStamped>& control_points) {
  static std::vector<geometry_msgs::msg::PoseStamped> path;
  path.clear();
  if (control_points.size() < 2) {
    return std::move(path);
  }

  double x0 = control_points.front().pose.position.x,
         y0 = control_points.front().pose.position.y;

  double x1 = control_points.back().pose.position.x,
         y1 = control_points.back().pose.position.y;

  RCLCPP_INFO(node_->get_logger(), "interpolation path (%f, %f), (%f,%f)", x0,
              y0, x1, y1);
  double dx = x0 - x1, dy = y0 - y1;
  double full_distance = hypot(dx, dy);
  int interpolation_num = static_cast<int>(
      full_distance / costmap_ros_->getCostmap()->getResolution());

  path.push_back(control_points.front());
  for (int i = 0; i < interpolation_num; ++i) {
    double x_t =
        (x0 * (interpolation_num - i) + x1 * i) / float(interpolation_num);
    double y_t =
        (y0 * (interpolation_num - i) + y1 * i) / float(interpolation_num);
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x_t;
    pose.pose.position.y = y_t;
    pose.pose.position.z = 0;
    path.push_back(pose);
  }
  path.push_back(control_points.back());
  return std::move(path);
}

}  // namespace mcr_global_planner
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mcr_global_planner::MCRSplinePlanner,
                       nav2_core::GlobalPlanner)
