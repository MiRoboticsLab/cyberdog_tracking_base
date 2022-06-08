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
#ifndef MCR_PLANNER_PLUGINS__POLYNOMIAL_INTERPOLATION_HPP_
#define MCR_PLANNER_PLUGINS__POLYNOMIAL_INTERPOLATION_HPP_
#include <vector>
#include <string>
#include <memory>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "mcr_global_planner/spliner.hpp"
// Eigen


namespace mcr_planner_plugins
{

class PolynomialInterpolation : public mcr_global_planner::Spliner
{
public:
  nav_msgs::msg::Path spline(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses)
  override;
  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap
  ) override;

protected:
  std::vector<geometry_msgs::msg::PoseStamped>&& interpolation(
    const std::vector<geometry_msgs::msg::PoseStamped>&control_points);
  void calculateT(double t0, double t1);

private:
  double det_ = 0.05;
  double max_speed_ = 1.0;
  double dist_scale_ = 1.0;
  nav2_util::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  Eigen::Matrix<double, 6, 6> T;
  Eigen::Matrix<double, 6, 1> A, B;
  Eigen::Matrix<double, 6, 1> X;
  Eigen::Matrix<double, 6, 1> Y;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace mcr_planner_plugins

#endif  // MCR_PLANNER_PLUGINS__POLYNOMIAL_INTERPOLATION_HPP_
