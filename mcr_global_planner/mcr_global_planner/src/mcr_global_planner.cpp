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
#include "mcr_global_planner/mcr_global_planner.hpp"
#include "mcr_global_planner/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/path_ops.hpp"
#include "nav2_util/node_utils.hpp"

namespace mcr_global_planner
{

MCRGlobalPlanner::MCRGlobalPlanner()
: calc_loader_("mcr_global_planner", "mcr_global_planner::PotentialCalculator"),
  traceback_loader_("mcr_global_planner", "mcr_global_planner::Traceback"),
  potential_grid_(HIGH_POTENTIAL), cached_goal_x_(-1), cached_goal_y_(-1),
  tf_(nullptr), costmap_(nullptr)
{
}

MCRGlobalPlanner::~MCRGlobalPlanner()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "Destroying plugin %s of type MCRGlobalPlanner", name_.c_str());
}


void MCRGlobalPlanner::configure(
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
    "configuring plugin %s of type MCRGlobalPlanner.", name_.c_str());

  potential_grid_.setInfo(infoFromCostmap(costmap_ros));

  cost_interpreter_ = std::make_shared<CostInterpreter>();
  cost_interpreter_->initialize(node_, name_, costmap_ros_);

  std::string plugin_name;

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".potential_calculator", rclcpp::ParameterValue(
      "mcr_planner_plugins::AStar"));
  node_->get_parameter(name_ + ".potential_calculator", plugin_name);
  RCLCPP_INFO(
    node_->get_logger(), "MCRGlobalPlanner Using PotentialCalculator \"%s\"",
    plugin_name.c_str());
  calculator_ = calc_loader_.createUniqueInstance(plugin_name);
  calculator_->initialize(node_, name_, costmap_ros_, cost_interpreter_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".traceback",
    rclcpp::ParameterValue("mcr_planner_plugins::GradientPath"));
  node_->get_parameter(name_ + ".traceback", plugin_name);
  RCLCPP_INFO(node_->get_logger(), "DluxGlobalPlanner Using Traceback \"%s\"", plugin_name.c_str());
  traceback_ = traceback_loader_.createUniqueInstance(plugin_name);
  traceback_->initialize(node_, name_, cost_interpreter_);


  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".path_caching", rclcpp::ParameterValue(
      false));
  node_->get_parameter(name_ + ".path_caching", path_caching_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".improvement_threshold", rclcpp::ParameterValue(
      -1.0));
  node_->get_parameter(name_ + ".improvement_threshold", improvement_threshold_);
  cached_path_cost_ = -1.0;

  bool publish_potential;

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".publish_potential", rclcpp::ParameterValue(
      false));
  node_->get_parameter(name_ + ".publish_potential", publish_potential);
  // if (publish_potential)
  //   potential_pub_.init(planner_nh, "potential_grid", "potential");
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".print_statistics", rclcpp::ParameterValue(
      false));
  node_->get_parameter(name_ + ".print_statistics", print_statistics_);
}

void MCRGlobalPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "Cleaning up plugin %s of type MCRGlobalPlanner",
    name_.c_str());
}

void MCRGlobalPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type MCRGlobalPlanner",
    name_.c_str());
}
void MCRGlobalPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type MCRGlobalPlanner",
    name_.c_str());
}

geometry_msgs::msg::Pose2D MCRGlobalPlanner::transformPose(
  const geometry_msgs::msg::PoseStamped & in_pose,
  const std::string frame)
{
  auto X = [](geometry_msgs::msg::PoseStamped & pose) -> geometry_msgs::msg::Pose2D
    {
      geometry_msgs::msg::Pose2D ret_pose;
      ret_pose.x = pose.pose.position.x;
      ret_pose.y = pose.pose.position.y;
      ret_pose.theta = tf2::getYaw(pose.pose.orientation);
      return ret_pose;
    };

  geometry_msgs::msg::PoseStamped out_pose;
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return X(out_pose);
  }

  try {
    tf_->transform(in_pose, out_pose, frame, tf2::durationFromSec(0.3));
    out_pose.header.frame_id = frame;
    return X(out_pose);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in transformPose: %s", ex.what());
  }
  return X(out_pose);
}


nav_msgs::msg::Path MCRGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const mcr_nav_grid::NavGridInfo & info = infoFromCostmap(costmap_ros_);

  if (potential_grid_.getInfo() != info) {
    potential_grid_.setInfo(info);
  }

  geometry_msgs::msg::Pose2D local_start = transformPose(start, potential_grid_.getFrameId());
  geometry_msgs::msg::Pose2D local_goal = transformPose(goal, potential_grid_.getFrameId());


  // Check Start / Goal Quality
  unsigned int x, y;
  if (!worldToGridBounded(info, local_start.x, local_start.y, x, y)) {
    cached_path_cost_ = -1.0;
    throw StartBoundsException(start);
  }
  if (costmap_->getCost(x, y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    cached_path_cost_ = -1.0;
    throw OccupiedStartException(start);
  }
  if (!worldToGridBounded(info, local_goal.x, local_goal.y, x, y)) {
    cached_path_cost_ = -1.0;
    throw GoalBoundsException(goal);
  }
  if (costmap_->getCost(x, y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    cached_path_cost_ = -1.0;
    throw OccupiedGoalException(goal);
  }

  bool cached_plan_available = false;
  if (path_caching_ && hasValidCachedPath(local_goal, x, y)) {
    if (shouldReturnCachedPathImmediately()) {
      return nav_2d_utils::pathToPath(cached_path_);
    }
    cached_plan_available = true;
  }

  // Commence path planning.
  unsigned int n_updated = calculator_->updatePotentials(potential_grid_, local_start, local_goal);
  // potential_pub_.publish();
  double path_cost = 0.0;  // right now we don't do anything with the cost
  nav_2d_msgs::msg::Path2D path = traceback_->getPath(
    potential_grid_,
    nav_2d_utils::poseToPose2D(start.pose),
    nav_2d_utils::poseToPose2D(goal.pose),
    path_cost);

  if (print_statistics_) {
    RCLCPP_INFO(
      node_->get_logger(), "DluxGlobalPlanner Got plan! Cost: %.2f, %d updated "
      "potentials, path of length with %zu poses.",
      path_cost, n_updated, path.poses.size());
  }

  // If there is a cached path available and the new path cost has not sufficiently improved
  if (cached_plan_available && !shouldReturnNewPath(path, path_cost)) {
    return nav_2d_utils::pathToPath(cached_path_);
  }
  cached_path_cost_ = path_cost;
  cached_path_ = path;
  return nav_2d_utils::pathToPath(path);
}

bool MCRGlobalPlanner::isPlanValid(const nav_2d_msgs::msg::Path2D & path) const
{
  // NB: Only checks for obstacles at each pose
  unsigned int x, y;
  const mcr_nav_grid::NavGridInfo & info = infoFromCostmap(costmap_ros_);
  for (geometry_msgs::msg::Pose2D pose : path.poses) {
    if (!worldToGridBounded(info, pose.x, pose.y, x, y) ||
      costmap_->getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE)
    {
      return false;
    }
  }
  return true;
}
bool MCRGlobalPlanner::hasValidCachedPath(
  const geometry_msgs::msg::Pose2D & /*local_goal*/,
  unsigned int goal_x, unsigned int goal_y)
{
  bool ret = cached_path_cost_ >= 0 && cached_goal_x_ == goal_x &&
    cached_goal_y_ == goal_y && isPlanValid(cached_path_);
  cached_goal_x_ = goal_x;
  cached_goal_y_ = goal_y;
  return ret;
}
bool MCRGlobalPlanner::shouldReturnCachedPathImmediately() const
{
  // If we don't care if the plan improves, return immediately.
  return improvement_threshold_ < 0.0;
}
bool MCRGlobalPlanner::shouldReturnNewPath(
  const nav_2d_msgs::msg::Path2D & /*new_path*/,
  const double new_path_cost) const
{
  return new_path_cost + improvement_threshold_ <= cached_path_cost_;
}

using rcl_interfaces::msg::ParameterType;
void MCRGlobalPlanner::on_parameter_event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  for (auto & changed_parameter : event->changed_parameters) {
    const auto & type = changed_parameter.value.type;
    const auto & name = changed_parameter.name;
    const auto & value = changed_parameter.value;

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".improvement_threshold") {
        improvement_threshold_ = value.double_value;
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".path_caching") {
        path_caching_ = value.bool_value;
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == name_ + ".traceback") {
        std::string plugin_name = value.string_value;
        RCLCPP_INFO(
          node_->get_logger(), "DluxGlobalPlanner Using Traceback \"%s\"", plugin_name.c_str());
        traceback_ = traceback_loader_.createUniqueInstance(plugin_name);
        traceback_->initialize(node_, name_, cost_interpreter_);
      } else if (name == name_ + ".potential_calculator") {
        std::string plugin_name = value.string_value;
        RCLCPP_INFO(
          node_->get_logger(), "MCRGlobalPlanner Using PotentialCalculator \"%s\"",
          plugin_name.c_str());
        calculator_ = calc_loader_.createUniqueInstance(plugin_name);
        calculator_->initialize(node_, name_, costmap_ros_, cost_interpreter_);
      }
    }
  }
}
}  // namespace mcr_global_planner
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mcr_global_planner::MCRGlobalPlanner, nav2_core::GlobalPlanner)
