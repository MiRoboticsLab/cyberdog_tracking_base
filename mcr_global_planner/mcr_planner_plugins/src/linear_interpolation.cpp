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

#include "mcr_planner_plugins/linear_interpolation.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"

namespace mcr_planner_plugins {

void LinearInterpolation::initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string& name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) {
  node_ = node;
  name_ = name;
  costmap_ros_ = costmap;

  double max_speed_ = 1.0;
  double dist_scale_ = 1.0;
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".max_speed",
                                               rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".max_speed", max_speed_);

  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".dist_scale",
                                               rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".dist_scale", dist_scale_);
}

nav_msgs::msg::Path LinearInterpolation::spline(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
  nav_msgs::msg::Path path;
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
LinearInterpolation::interpolation(
    const std::vector<geometry_msgs::msg::PoseStamped>& control_points) {
  double max_speed = max_speed_;
  static std::vector<geometry_msgs::msg::PoseStamped> path;
  path.clear();
  if (control_points.size() < 2) {
    return std::move(path);
  }

  double x0 = control_points.front().pose.position.x,
         y0 = control_points.front().pose.position.y;

  double x1 = control_points.back().pose.position.x,
         y1 = control_points.back().pose.position.y;

  double dx = x0 - x1, dy = y0 - y1;
  double full_distance = hypot(dx, dy);
  int interpolation_num = static_cast<int>(
      full_distance / costmap_ros_->getCostmap()->getResolution());

  for (int i = 0; i < interpolation_num; ++i) {
    double x_t =
        (x0 * (interpolation_num - i) + x1 * i) / float(interpolation_num);
    double y_t =
        (y0 * (interpolation_num - i) + y1 * i) / float(interpolation_num);
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x_t;
    pose.pose.position.y = y_t;
    path.push_back(pose);
  }

  return std::move(path);
}
}  //  namespace mcr_planner_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mcr_planner_plugins::LinearInterpolation,
                       mcr_global_planner::Spliner)
