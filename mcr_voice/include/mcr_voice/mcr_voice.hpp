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
#ifndef MCR_VOICE__MCR_VOICE_HPP_
#define MCR_VOICE__MCR_VOICE_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "mcr_msgs/action/detail/target_tracking__struct.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "protocol/msg/audio_play.hpp"

namespace mcr_voice
{
/**
 * @class mcr_voice::MCRVoice
 * @brief a transform for uwb raw data to posestamp
 */
class MCRVoice : public rclcpp::Node
{
public:
  /**
   * @brief A constructor for mcr_planner::MCRVoice
   */
  MCRVoice();
  /**
   * @brief A destructor for mcr_planner::MCRVoice
   */
  ~MCRVoice();

protected:
  /**
   * @brief call audio play server 
   * @param audio content to play
   */
  void playAudio(const std::string& audio);
  /**
   * @brief subscribe action server's feedback for audio tracking status
   * @param feedback for tracking status
   */
  void incomingFeedback(mcr_msgs::action::TargetTracking_FeedbackMessage::ConstSharedPtr feedback);
private:
  rclcpp::Subscription<mcr_msgs::action::TargetTracking_FeedbackMessage>::SharedPtr feedback_sub_;
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_play_client_;
  double valid_range_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace mcr_voice

#endif  // MCR_VOICE__MCR_VOICE_HPP_
