// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#include "mcr_tracking_components/dwb_critics/keep_person_insight.hpp"

#include <string>
#include <vector>

#include "angles/angles.h"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/robot_utils.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mcr_tracking_components::KeepPersonInsightCritic,
                       dwb_core::TrajectoryCritic)

namespace mcr_tracking_components {

inline double hypot_sq(double dx, double dy) { return dx * dx + dy * dy; }

void KeepPersonInsightCritic::onInit() {
  normal_sacle_ = 0.0;
  inuse_ = true;
  rcl_node_ = node_.lock();
  if (!rcl_node_) {
    throw std::runtime_error{"Failed to lock node"};
  }
  nav2_util::declare_parameter_if_not_declared(rcl_node_, dwb_plugin_name_ + "." + name_ + ".target_topic", 
    rclcpp::ParameterValue("person"));
  rcl_node_->get_parameter(dwb_plugin_name_ + "." + name_ + ".target_topic", target_topic_);

  lookahead_time_ = nav_2d_utils::searchAndGetParam(
      rcl_node_, dwb_plugin_name_ + "." + name_ + ".lookahead_time", -1.0);


  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rcl_node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    rcl_node_->get_node_base_interface(),
    rcl_node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  pose_sub_ = rcl_node_->create_subscription<protocol::msg::Person>(
    target_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&KeepPersonInsightCritic::poseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(
      rcl_node_->get_logger(),
      "Keep target insight critic subscribed to tracking pose: tracking_pose");

  reset();
  normal_sacle_ = getScale();
}

void KeepPersonInsightCritic::reset() {}

bool KeepPersonInsightCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & /*vel*/,
  const geometry_msgs::msg::Pose2D & /*goal*/,
  const nav_2d_msgs::msg::Path2D &)
{
  cur_yaw_ = pose.theta;
  goal_yaw_ = (320.0 - (latest_person_.track_res.roi.x_offset + latest_person_.track_res.roi.width / 2.0)) * 0.0027;
  return true;
}

double KeepPersonInsightCritic::scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D &traj) {
  if((rcl_node_->now().seconds() - latest_timestamp_.seconds()) > 2.5){
    // RCLCPP_INFO(rcl_node_->get_logger(),
                // "Keep person insight critic timed out for subscriber: person");
    return 0.0;
  }

  return scoreRotation(traj);
}

double KeepPersonInsightCritic::scoreRotation(
    const dwb_msgs::msg::Trajectory2D &traj) {
  if (traj.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory.");
  }

  double end_yaw;
  if (lookahead_time_ >= 0.0) {
    geometry_msgs::msg::Pose2D eval_pose =
        dwb_core::projectPose(traj, lookahead_time_);
    end_yaw = eval_pose.theta;
  } else {
    end_yaw = traj.poses.back().theta;
  }
  return fabs(angles::shortest_angular_distance(end_yaw - cur_yaw_, goal_yaw_));
}

void KeepPersonInsightCritic::poseCallback(const protocol::msg::Person::SharedPtr msg)
{
  if(msg->track_res.roi.height == 0 || msg->track_res.roi.width == 0) return;

  latest_person_ = *msg;
  latest_timestamp_ = rcl_node_->now();
}

}  // namespace mcr_tracking_components
