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

#ifndef MCR_PLANNER_PLUGINS__B_SPLINER_HPP_
#define MCR_PLANNER_PLUGINS__B_SPLINER_HPP_

#include <vector>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mcr_global_planner/spliner.hpp"

namespace mcr_planner_plugins
{

class BSpliner : public mcr_global_planner::Spliner
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
  double deboor_cox(int i, int k, double u);

private:
  double det_ = 0.05;
  std::vector<double> knots_vector_;
  nav2_util::LifecycleNode::SharedPtr node_;
  int k_ = 4;
  int flag_ = 2;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};
}   //  namespace mcr_planner_plugins

#endif  //  MCR_PLANNER_PLUGINS__B_SPLINER_HPP_
