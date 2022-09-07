// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#ifndef mcr_tracking_components__PLUGINS__DECORATOR__TARGET_UPDATER_NODE_HPP_
#define mcr_tracking_components__PLUGINS__DECORATOR__TARGET_UPDATER_NODE_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <deque>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "behaviortree_cpp_v3/decorator_node.h"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "mcr_tracking_components/behavior_tree_nodes/orientation_derivers.hpp"

namespace mcr_tracking_components
{
static double poseDistanceSq(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  double dx = p2.position.x - p1.position.x;
  double dy = p2.position.y - p1.position.y;
  return dx * dx + dy * dy;
}
/**
 * @brief A BT::DecoratorNode that subscribes to a goal topic and updates
 * the current goal on the blackboard
 */
class TargetUpdater : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for mcr_tracking_components::TargetUpdater
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TargetUpdater(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Original Goal"),
      BT::InputPort<unsigned char>("input_tracking_mode", "The true mode after decision"),
      BT::OutputPort<double>("distance", "Distance between target and robot."),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "output_goal",
        "Received Goal by subscription"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "output_goals",
        "Received Goals by subscription"),
      BT::OutputPort<unsigned int>(
        "output_exception_code",
        "Exception code for the real reason."),
    };
  }

protected:
  double det_ = 0.05;
  std::vector<std::vector<double>> cheat_sheet_;
  nav_msgs::msg::Path spline(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses);
  std::vector<geometry_msgs::msg::PoseStamped>&& bezier(
    const std::vector<geometry_msgs::msg::PoseStamped>&control_points);

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Callback function for goal update topic
   * @param msg Shared pointer to geometry_msgs::msg::PoseStamped message
   */
  void callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  geometry_msgs::msg::PoseStamped translatePoseByMode(const geometry_msgs::msg::PoseStamped & pose);

  void historyPoseUpdate(const geometry_msgs::msg::PoseStamped & pose);

  void truncatOverduePosesAndAppendCurPose();
  void truncatHinderPosesAndAppendCurPose();
  void checkAndDerivateAngle();
  void publishPoses(const std::deque<geometry_msgs::msg::PoseStamped> & poses);
  bool isValid(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformed_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spline_poses_pub_;
  geometry_msgs::msg::PoseStamped last_goal_received_;
  geometry_msgs::msg::PoseStamped last_goal_transformed_;
  rclcpp::Time latest_timestamp_;

  std::mutex mutex_;
  unsigned char current_mode_ = 255;

  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<geometry_msgs::msg::PoseStamped> historical_poses_;
  int max_pose_inuse_;
  double dist_sq_throttle_, overtime_;
  float distance_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  pluginlib::ClassLoader<OrientationDeriver> deriver_loader_;  
  OrientationDeriver::Ptr orientation_deriver_;
};

}  // namespace mcr_tracking_components

#endif  // mcr_tracking_components__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_
