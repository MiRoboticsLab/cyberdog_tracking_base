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

#ifndef mcr_tracking_components__PLUGINS__DECORATOR__CHARGER_UPDATER_NODE_HPP_
#define mcr_tracking_components__PLUGINS__DECORATOR__CHARGER_UPDATER_NODE_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <visualization_msgs/msg/marker.hpp>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
namespace mcr_tracking_components {

/**
 * @brief A BT::DecoratorNode that subscribes to a goal topic and updates
 * the current goal on the blackboard
 */
class ChargerUpdater : public BT::DecoratorNode {
 public:
  /**
   * @brief A constructor for mcr_tracking_components::ChargerUpdater
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ChargerUpdater(const std::string& xml_tag_name,
                 const BT::NodeConfiguration& conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() {
    return {
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
            "output_goals", "Received Goals by subscription"),
        BT::OutputPort<unsigned int>("output_exception_code",
                                     "Exception code for the real reason."),
    };
  }

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
  void callback_updated_goal(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publishPoses(const std::deque<geometry_msgs::msg::PoseStamped>& poses);
  bool isValid(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  geometry_msgs::msg::PoseStamped translatePose(
      const geometry_msgs::msg::PoseStamped& pose);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      transformed_pose_pub_;
  geometry_msgs::msg::PoseStamped last_goal_received_;
  geometry_msgs::msg::PoseStamped last_goal_transformed_;
  rclcpp::Time latest_timestamp_;

  std::mutex mutex_;

  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<geometry_msgs::msg::PoseStamped> historical_poses_;
  int max_pose_inuse_;
  float distance_;
  double offset_x_, offset_y_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}  // namespace mcr_tracking_components

#endif  // mcr_tracking_components__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_
