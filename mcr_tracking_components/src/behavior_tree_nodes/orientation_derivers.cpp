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

#include "mcr_tracking_components/behavior_tree_nodes/orientation_derivers.hpp"
#include "mcr_tracking_components/behavior_tree_nodes/target_updater_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_core/exceptions.hpp"

namespace mcr_tracking_components
{

void MeanOrientationDeriver::initialize(const rclcpp::Node::SharedPtr node,
                                        const std::string &global_frame, 
                                        const std::shared_ptr<tf2_ros::Buffer> tf_buffer){
  OrientationDeriver::initialize(node, global_frame, tf_buffer);
}

geometry_msgs::msg::PoseStamped 
MeanOrientationDeriver::deriveOrientation(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

  geometry_msgs::msg::PoseStamped msg_with_orientation, msg_with_orientation1;

  //该模式下的跟随位姿对应在全局坐标系上的位姿
  if (!nav2_util::transformPoseInTargetFrame(
      *msg, msg_with_orientation, *tf_buffer_,
      global_frame_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Faild transform target pose to %s", global_frame_.c_str());
    throw nav2_core::TFException("Transformed error in target updater node");
  }

  //push pose when robot has moved a little.
  if (sqrt(poseDistanceSq(historical_raw_poses_.front().pose, msg_with_orientation.pose)) > 0.3) {
    historical_raw_poses_.push_front(msg_with_orientation);
  }
  // deal with pose's orientation
  while (historical_raw_poses_.size() > 4) {
    historical_raw_poses_.pop_back();
  }
  double theta = 0.0;

  int k = 0;
  for (auto it = historical_raw_poses_.begin() + 1; it < historical_raw_poses_.end(); it++) {
    double t = atan2(
      msg_with_orientation.pose.position.y - it->pose.position.y,
      msg_with_orientation.pose.position.x - it->pose.position.x);
    if(t > -M_PI && t < -M_PI_2) t += 2 * M_PI;
    theta += t;
    k += 1;
  }
  theta = theta / k;

  msg_with_orientation.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);

  if (!nav2_util::transformPoseInTargetFrame(
      msg_with_orientation, msg_with_orientation1, *tf_buffer_,
      msg->header.frame_id))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Faild transform target pose to %s", msg->header.frame_id.c_str());
    throw nav2_core::TFException("Transformed error in target updater node");
  }

  msg_with_orientation1.pose.position = msg->pose.position;
  return msg_with_orientation1;
}


} // namespace mcr_tracking_components

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mcr_tracking_components::MeanOrientationDeriver,
  mcr_tracking_components::OrientationDeriver)