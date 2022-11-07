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
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"

#include "mcr_voice/mcr_voice.hpp"

using namespace std::chrono_literals;

namespace mcr_voice
{

MCRVoice::MCRVoice()
: nav2_util::LifecycleNode("mcr_voice", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("feedback_topic", "tracking_target/_action/feedback");
  declare_parameter("valid_range", 3.5);
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // get_parameter("planner_plugins", planner_ids_);
 
}

MCRVoice::~MCRVoice()
{
}

nav2_util::CallbackReturn
MCRVoice::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  auto node = shared_from_this();
  std::string feedback_topic, produced_pose;
  get_parameter("feedback_topic", feedback_topic);
  get_parameter("valid_range", valid_range_);

  // Initialize pubs & subs
  // pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(produced_pose, 5);

    audio_play_client_ = create_client<protocol::srv::AudioTextPlay>(
      "speech_text_play",
      rmw_qos_profile_services_default,
      callback_group_);

  feedback_sub_ = create_subscription<mcr_msgs::action::TargetTracking_FeedbackMessage>(feedback_topic, 10, 
                                                        std::bind(&MCRVoice::incomingFeedback, this, std::placeholders::_1));



  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MCRVoice::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // pose_pub_->on_activate();
  // feedback_sub_->on_activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MCRVoice::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // pose_pub_->on_deactivate();
  // feedback_sub_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MCRVoice::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // pose_pub_.reset();
  feedback_sub_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MCRVoice::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void MCRVoice::playAudio(const std::string& audio){
  auto request = std::make_shared<protocol::srv::AudioTextPlay::Request>();
  request->module_name = get_name();
  request->is_online = true;
  request->text = audio;
  auto callback = [&](rclcpp::Client<protocol::srv::AudioTextPlay>::SharedFuture future) {
      RCLCPP_INFO(get_logger(), "Audio play result: %s", future.get()->status == 0 ? "success" : "failed");
    };
  auto future_online = audio_play_client_->async_send_request(request, callback);
  if (future_online.wait_for(std::chrono::milliseconds(3000)) == std::future_status::timeout) {
    RCLCPP_ERROR(get_logger(), "Cannot get response of AudioPlay");
  }
}



void
MCRVoice::incomingFeedback(mcr_msgs::action::TargetTracking_FeedbackMessage::ConstSharedPtr feedbackmsg)
{
  if(feedbackmsg->feedback.exception_code == nav2_core::DETECTOREXCEPTION){
    RCLCPP_INFO(get_logger(), "The target is out of sight. Voice prompts are needed.");  
    playAudio("主人你在哪里，我好像迷路了。");
  }
  
  if(feedbackmsg->feedback.current_distance > valid_range_){
    RCLCPP_INFO(get_logger(), "It is too far away from the target and needs voice prompt.");  
    playAudio("主人，等等我，跟不上你了。");
  }
 
   if(feedbackmsg->feedback.exception_code == nav2_core::PLANNEREXECPTION || 
    feedbackmsg->feedback.exception_code == nav2_core::CONTROLLEREXECPTION){
    RCLCPP_INFO(get_logger(), "Something is wrong with planning. Voice prompts are needed.");  
    playAudio("主人，我遇到了一些困难，帮帮我好吗？");
  }
}

}  // namespace mcr_planner
