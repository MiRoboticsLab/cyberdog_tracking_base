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

#ifndef mcr_tracking_components__PLUGINS__ORIENTATION_DERIVERS_HPP_
#define mcr_tracking_components__PLUGINS__ORIENTATION_DERIVERS_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <deque>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "behaviortree_cpp_v3/decorator_node.h"
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "Eigen/QR"

namespace mcr_tracking_components
{
class OrientationDeriver{
public:    
  using Ptr = std::shared_ptr<mcr_tracking_components::OrientationDeriver>;
  virtual ~OrientationDeriver(){}
  virtual void initialize(const rclcpp::Node::SharedPtr node,
                          const std::string &global_frame, 
                          const std::shared_ptr<tf2_ros::Buffer> tf_buffer){
    node_ = node;
    global_frame_ = global_frame;
    tf_buffer_ = tf_buffer;
    
  }
  virtual geometry_msgs::msg::PoseStamped deriveOrientation(const geometry_msgs::msg::PoseStamped::SharedPtr msg)=0;
protected:
  rclcpp::Node::SharedPtr node_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::deque<geometry_msgs::msg::PoseStamped> historical_raw_poses_;
};


class MeanOrientationDeriver : public OrientationDeriver{
public:
  void initialize(const rclcpp::Node::SharedPtr node, 
                  const std::string &global_frame, 
                  const std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
  geometry_msgs::msg::PoseStamped deriveOrientation(const geometry_msgs::msg::PoseStamped::SharedPtr msg) override;
};

#include "nav_msgs/msg/path.hpp"
class FittingOrientationDeriver : public OrientationDeriver{
public:
  void initialize(const rclcpp::Node::SharedPtr node, 
                  const std::string &global_frame, 
                  const std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
  geometry_msgs::msg::PoseStamped deriveOrientation(const geometry_msgs::msg::PoseStamped::SharedPtr msg) override;
private:
  nav_msgs::msg::Path spline(const std::vector<geometry_msgs::msg::PoseStamped> & poses);
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

};

class KalmanOrientationDeriver : public OrientationDeriver{
public:
  void initialize(const rclcpp::Node::SharedPtr node, 
                  const std::string &global_frame, 
                  const std::shared_ptr<tf2_ros::Buffer> tf_buffer) override;
  geometry_msgs::msg::PoseStamped deriveOrientation(const geometry_msgs::msg::PoseStamped::SharedPtr msg) override;

private:
  Eigen::Matrix<double, 4, 1> x_, U_;   // initial state, external motion.
  Eigen::Matrix4d P_, F_, I_;  // initial uncertainty, next state function, identity matrix.
  Eigen::Matrix<double, 2, 4> H_; // measurement function.
  Eigen::Matrix2d R_; // measurement uncertainty.

};

}  // namespace mcr_tracking_components

#endif  // mcr_tracking_components__PLUGINS__ORIENTATION_DERIVERS_HPP_
