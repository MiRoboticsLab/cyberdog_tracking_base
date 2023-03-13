// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <nav2_util/node_utils.hpp>
#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"

#include "mcr_uwb/mcr_uwb.hpp"

using namespace std::chrono_literals;

namespace mcr_uwb
{

MCRUwb::MCRUwb()
: nav2_util::LifecycleNode("mcr_uwb", "", true),
pose_history_duration_(rclcpp::Duration::from_seconds(0.5))
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("uwb_data", "uwb_raw");
  declare_parameter("produced_pose", "tracking_pose");
  declare_parameter("buffer_seconds", 1.0);

  // get_parameter("planner_plugins", planner_ids_);
 
}

MCRUwb::~MCRUwb()
{
}

nav2_util::CallbackReturn
MCRUwb::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  auto node = shared_from_this();
  std::string uwb_data_topic, produced_pose;
  get_parameter("uwb_data", uwb_data_topic);
  get_parameter("produced_pose", produced_pose);
  double buffer_seconds;
  get_parameter("buffer_seconds", buffer_seconds);
  pose_history_duration_ = rclcpp::Duration::from_seconds(buffer_seconds);
  

  // Initialize pubs & subs
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(produced_pose, 5);

  uwb_sub_ = create_subscription<protocol::msg::UwbRaw>(uwb_data_topic, 10, 
                                                        std::bind(&MCRUwb::incomingUwb, this, std::placeholders::_1));



  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MCRUwb::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  pose_cumulate_.pose.position.x = 0.0;
  pose_cumulate_.pose.position.y = 0.0;
  pose_cumulate_.pose.position.z = 0.0;

  pose_pub_->on_activate();
  // uwb_sub_->on_activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MCRUwb::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();
  // uwb_sub_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MCRUwb::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  pose_pub_.reset();
  uwb_sub_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MCRUwb::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

geometry_msgs::msg::PoseStamped MCRUwb::dynamic_window_ave_filter(const geometry_msgs::msg::PoseStamped& pose) {
  geometry_msgs::msg::PoseStamped pose_stamp;
  auto current_time = now();
  pose_stamp.header.stamp = current_time;
  if (!pose_history_.empty()) {
    auto front_time = rclcpp::Time(pose_history_.front().header.stamp);
    while (current_time - front_time > pose_history_duration_) {
      const auto & front_pose = pose_history_.front();
      pose_cumulate_.pose.position.x -= front_pose.pose.position.x;
      pose_cumulate_.pose.position.y -= front_pose.pose.position.y;
      pose_cumulate_.pose.position.z -= front_pose.pose.position.z;
      pose_history_.pop_front();
      if (pose_history_.empty()) {
        break;
      }
      front_time = rclcpp::Time(pose_history_.front().header.stamp);
    }
  }
  pose_stamp.pose.position.x = pose.pose.position.x;
  pose_stamp.pose.position.y = pose.pose.position.y;
  pose_history_.push_back(pose_stamp);

  const auto back_pose = pose_history_.back();
  pose_cumulate_.pose.position.x += back_pose.pose.position.x;
  pose_cumulate_.pose.position.y += back_pose.pose.position.y;
  
  pose_stamp.pose.position.x = pose_cumulate_.pose.position.x / pose_history_.size();
  pose_stamp.pose.position.y = pose_cumulate_.pose.position.y / pose_history_.size();
  pose_stamp.pose.orientation.w = 1.0;
  return pose_stamp;
}

void
MCRUwb::incomingUwb(protocol::msg::UwbRaw::ConstSharedPtr uwb)
{
  if(!pose_pub_->is_activated()){
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header = uwb->header;

  RCLCPP_DEBUG(get_logger(), "received uwb raw data, frame id: %s, dist: %f, angle: %f.", uwb->header.frame_id.c_str(), uwb->dist, uwb->angle);

  pose.pose.position.x = uwb->dist * cos(-uwb->angle);
  pose.pose.position.y = uwb->dist * sin(-uwb->angle);

  pose_pub_->publish(std::move(dynamic_window_ave_filter(pose)));
}

}  // namespace mcr_planner
