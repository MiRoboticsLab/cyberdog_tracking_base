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

#include <string>
#include <memory>
#include "nav2_core/exceptions.hpp"
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "mcr_msgs/action/target_tracking.hpp"
#include <mcr_tracking_components/behavior_tree_nodes/target_updater_node.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "Eigen/Dense"
#include "Eigen/QR"


namespace mcr_tracking_components
{

using std::placeholders::_1;
nav_msgs::msg::Path spline(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses);
TargetUpdater::TargetUpdater(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  distance_ = 0.0;
  std::string goal_updater_topic;

  nav2_util::declare_parameter_if_not_declared(
    node_, "global_frame",
    rclcpp::ParameterValue("map"));
  node_->get_parameter("global_frame", global_frame_);

  nav2_util::declare_parameter_if_not_declared(
    node_, "goal_updater_topic",
    rclcpp::ParameterValue("tracking_pose"));
  node_->get_parameter("goal_updater_topic", goal_updater_topic);

  nav2_util::declare_parameter_if_not_declared(
    node_, "max_pose_inuse", rclcpp::ParameterValue(5));
  node_->get_parameter("max_pose_inuse", max_pose_inuse_);

  nav2_util::declare_parameter_if_not_declared(
    node_, "dist_throttle", rclcpp::ParameterValue(0.3));
  node_->get_parameter("dist_throttle", dist_sq_throttle_);
  dist_sq_throttle_ *= dist_sq_throttle_;

  nav2_util::declare_parameter_if_not_declared(
    node_, "overtime", rclcpp::ParameterValue(6.0));
  node_->get_parameter("overtime", overtime_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_updater_topic, rclcpp::SensorDataQoS(),
    std::bind(&TargetUpdater::callback_updated_goal, this, _1),
    sub_option);

  transformed_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    goal_updater_topic + "_transformed", rclcpp::SensorDataQoS());

  plan_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("plan_derived", 1);

  historical_poses_.clear();
  spline_poses_pub_ =
    node_->create_publisher<visualization_msgs::msg::Marker>(
    "spline_poses_traj",
    rclcpp::QoS(
      rclcpp::KeepLast(
        1)).transient_local().reliable());

  last_goal_received_.header.frame_id = global_frame_;
}

inline BT::NodeStatus TargetUpdater::tick()
{
  callback_group_executor_.spin_some();
  std::lock_guard<std::mutex> guard(mutex_);
  geometry_msgs::msg::PoseStamped goal;
  setOutput("output_exception_code", nav2_core::NOEXCEPTION);

  getInput("input_goal", goal);
  getInput("input_tracking_mode", current_mode_);

  if ((node_->now().seconds() - latest_timestamp_.seconds()) > overtime_) {
    RCLCPP_WARN(node_->get_logger(), "The target pose may be lost. %lf", overtime_);
    historical_poses_.clear();
    setOutput("output_exception_code", nav2_core::DETECTOREXCEPTION);
    config().blackboard->set<int>("exception_code", nav2_core::DETECTOREXCEPTION);
    return BT::NodeStatus::FAILURE;
  }

  if (rclcpp::Time(last_goal_received_.header.stamp) > rclcpp::Time(goal.header.stamp)) {
    goal = last_goal_received_;
    // goal = historical_poses_.front();
  }

  if (node_->count_subscribers(spline_poses_pub_->get_topic_name()) > 0) {
    publishPoses(historical_poses_);
  }
  config().blackboard->set<float>("distance", distance_);
  setOutput("distance", distance_);
  setOutput("output_goal", goal);
  if (historical_poses_.size() >= 2) {
    setOutput(
      "output_goals",
      std::vector<geometry_msgs::msg::PoseStamped>(
        historical_poses_.rbegin(),
        historical_poses_.rend()));
  }


  BT::NodeStatus status = child_node_->executeTick();

  if (status == BT::NodeStatus::FAILURE) {
    setOutput("output_exception_code", nav2_core::PLANNEREXECPTION);
    config().blackboard->set<int>("exception_code", nav2_core::PLANNEREXECPTION);
  }
  config().blackboard->set<int>("exception_code", nav2_core::NOEXCEPTION);

  return status;
}


geometry_msgs::msg::PoseStamped
TargetUpdater::translatePoseByMode(const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::PoseStamped tpose = pose;
  transform.header = pose.header;
  transform.child_frame_id = pose.header.frame_id;

  switch (current_mode_) {

    case mcr_msgs::action::TargetTracking::Goal::LEFT: {
        //左侧 1m
        double yaw = tf2::getYaw(pose.pose.orientation);
        transform.transform.translation.x = cos(yaw + 3.14 / 2);
        transform.transform.translation.y = sin(yaw + 3.14 / 2);
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        break;
      }
    case mcr_msgs::action::TargetTracking::Goal::RIGHT:    {
        //右侧 1m
        double yaw = tf2::getYaw(pose.pose.orientation);
        transform.transform.translation.x = cos(yaw - 3.14 / 2);
        transform.transform.translation.y = sin(yaw - 3.14 / 2);
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        break;
      }
    case mcr_msgs::action::TargetTracking::Goal::BEHIND:
    default:
      {
        // 后方1m
        // double yaw = tf2::getYaw(pose.pose.orientation);
        double yaw = atan2(pose.pose.position.y, pose.pose.position.x);
        transform.transform.translation.x = -1.0 * cos(yaw);
        transform.transform.translation.y = -1.0 * sin(yaw);
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        tf2::Quaternion qtn;
        qtn.setRPY(0,0,yaw);  // 单位是弧度
        tpose.pose.orientation.x = qtn.getX();
        tpose.pose.orientation.y = qtn.getY();
        tpose.pose.orientation.z = qtn.getZ();
        tpose.pose.orientation.w = qtn.getW();
        break;
      }
  }
  //当前跟随模式对应的方位
  geometry_msgs::msg::PoseStamped transformed_pose;
  tf2::doTransform(tpose, transformed_pose, transform);

  //该模式下的跟随位姿对应在全局坐标系上的位姿
  geometry_msgs::msg::PoseStamped global_frame_pose;

  if (!nav2_util::transformPoseInTargetFrame(
      transformed_pose, global_frame_pose, *tf_buffer_,
      global_frame_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Faild transform target pose to %s", global_frame_.c_str());
    throw nav2_core::TFException("Transformed error in target updater node");
  }

  return global_frame_pose;
}

double poseDistanceSq(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  double dx = p2.position.x - p1.position.x;
  double dy = p2.position.y - p1.position.y;
  return dx * dx + dy * dy;
}

bool TargetUpdater::isValid(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose_based_on_global_frame;
  try {
    pose_based_on_global_frame = tf_buffer_->transform(
      *msg, global_frame_, tf2::durationFromSec(0.2));
  } catch (...) {
    RCLCPP_WARN(
      node_->get_logger(), "failed to transform pose from %s to %s.",
      msg->header.frame_id.c_str(),
      global_frame_.c_str());
    return false;
  }
  if (poseDistanceSq(last_goal_received_.pose, pose_based_on_global_frame.pose) <
    dist_sq_throttle_)
  {
    return false;
  }
  last_goal_received_ = pose_based_on_global_frame;
  return true;
}


geometry_msgs::msg::PoseStamped TargetUpdater::derivedOrientation(
  geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

  geometry_msgs::msg::PoseStamped msg_with_orientation;


  //该模式下的跟随位姿对应在全局坐标系上的位姿
  if (!nav2_util::transformPoseInTargetFrame(
      *msg, msg_with_orientation, *tf_buffer_,
      global_frame_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Faild transform target pose to %s", global_frame_.c_str());
    throw nav2_core::TFException("Transformed error in target updater node");
  }

  //push pose when robot has moved a little.
  historical_raw_poses_.push_front(msg_with_orientation);
  // deal with pose's orientation
  while (historical_raw_poses_.size() > 15 &&
    poseDistanceSq(historical_raw_poses_.front().pose, historical_raw_poses_.back().pose) > 0.8)
  {
    historical_raw_poses_.pop_back();
  }
  double theta = 0.0;
  nav_msgs::msg::Path p = spline(
    std::vector<geometry_msgs::msg::PoseStamped>(
      historical_raw_poses_.rbegin(),
      historical_raw_poses_.rend()));
  size_t len = p.poses.size();
  p.header.frame_id = global_frame_;
  p.header.stamp = node_->now();
  plan_publisher_->publish(p);
  if (len > 2) {
    if (poseDistanceSq(p.poses.front().pose, msg_with_orientation.pose) <
      poseDistanceSq(p.poses.back().pose, msg_with_orientation.pose))
    {
      theta = atan2(
        p.poses[0].pose.position.y - p.poses[1].pose.position.y,
        p.poses[0].pose.position.x - p.poses[1].pose.position.x);
    } else {
      theta = atan2(
        p.poses[len - 1].pose.position.y - p.poses[len - 2].pose.position.y,
        p.poses[len - 1].pose.position.x - p.poses[len - 2].pose.position.x);
    }

    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, theta);    // 单位是弧度
    msg_with_orientation.pose.orientation.x = qtn.getX();
    msg_with_orientation.pose.orientation.y = qtn.getY();
    msg_with_orientation.pose.orientation.z = qtn.getZ();
    msg_with_orientation.pose.orientation.w = qtn.getW();
  }
  return msg_with_orientation;
}


void
TargetUpdater::callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

  std::lock_guard<std::mutex> guard(mutex_);
  latest_timestamp_ = node_->now();
  distance_ = hypot(msg->pose.position.x, msg->pose.position.y);
  if (!isValid(msg)) {
    return;
  }

  last_goal_transformed_ = translatePoseByMode(*msg);
  //visu debug info
  if (transformed_pose_pub_ != nullptr &&
    node_->count_subscribers(transformed_pose_pub_->get_topic_name()) > 0)
  {
    transformed_pose_pub_->publish(last_goal_transformed_);
  }

  historyPoseUpdate(last_goal_transformed_);
}


void TargetUpdater::historyPoseUpdate(const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped pose_based_on_global_frame;

  geometry_msgs::msg::PoseStamped pose_origin = pose;
  // pose_origin.header.stamp = node_->now();

  try {
    pose_based_on_global_frame = tf_buffer_->transform(
      pose_origin, global_frame_, tf2::durationFromSec(
        0.2));
  } catch (...) {
    RCLCPP_WARN(
      node_->get_logger(), "failed to transform pose from %s to %s.",
      pose_origin.header.frame_id.c_str(),
      global_frame_.c_str());
    return;
  }

  //deque front pose is the robot's latest pose.
  if (historical_poses_.empty()) {
    historical_poses_.push_front(std::move(pose_based_on_global_frame));
  } else {
    while (historical_poses_.size() > static_cast<size_t>(max_pose_inuse_)) {
      historical_poses_.pop_back();
    }
    //push pose when robot has moved a little.
    const geometry_msgs::msg::PoseStamped & latest_odom_pose = historical_poses_.front();
    if (poseDistanceSq(
        latest_odom_pose.pose,
        pose_based_on_global_frame.pose) > dist_sq_throttle_)
    {
      historical_poses_.push_front(pose_based_on_global_frame);
    }
  }

  //truncat redundant poses when the totle length is overflow.
  truncatHinderPosesAndAppendCurPose();
  checkAndDerivateAngle();
  publishPoses(historical_poses_);
}


void TargetUpdater::publishPoses(const std::deque<geometry_msgs::msg::PoseStamped> & poses)
{
  if (poses.empty()) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = global_frame_;
  marker.header.stamp = node_->now();
  marker.ns = "spline_poses";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(2, 0);
  marker.pose.orientation.w = 1.0;

  for (std::size_t i = 0; i < poses.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = poses[i].pose.position.x;
    point.y = poses[i].pose.position.y;
    point.z = 0;
    marker.points.push_back(point);
  }

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  spline_poses_pub_->publish(marker);
}


void TargetUpdater::truncatOverduePosesAndAppendCurPose()
{
  size_t len = historical_poses_.size();
  size_t guidance_index;
  geometry_msgs::msg::PoseStamped pose_based_on_global_frame;

  if (!nav2_util::getCurrentPose(pose_based_on_global_frame, *tf_buffer_, global_frame_)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "truncat overdue poses failed to obtain current pose based on map coordinate system.");
    return;
  }

  geometry_msgs::msg::PoseStamped guidance_pose;
  if (len < 4) {
    guidance_index = 0;
    guidance_pose = historical_poses_.front();
  } else {
    guidance_index = len / 2;
    guidance_pose = historical_poses_.at(guidance_index);
  }

  while (historical_poses_.size() > guidance_index) {
    double p1_x = historical_poses_.back().pose.position.x -
      pose_based_on_global_frame.pose.position.x;
    double p1_y = historical_poses_.back().pose.position.y -
      pose_based_on_global_frame.pose.position.y;
    double p2_x = guidance_pose.pose.position.x - pose_based_on_global_frame.pose.position.x;
    double p2_y = guidance_pose.pose.position.y - pose_based_on_global_frame.pose.position.y;
    if (p1_x * p2_x + p1_y * p2_y <= 0) {
      historical_poses_.pop_back();
    } else {
      break;
    }
  }
  if (poseDistanceSq(
      historical_poses_.back().pose,
      pose_based_on_global_frame.pose) < dist_sq_throttle_)
  {
    historical_poses_.pop_back();
  }
  historical_poses_.push_back(pose_based_on_global_frame);

}

void TargetUpdater::checkAndDerivateAngle()
{
  if (historical_poses_.size() < 2) {return;}
  for (size_t i = 0; i < historical_poses_.size() - 1; i++) {
    geometry_msgs::msg::PoseStamped & latest_pose = historical_poses_[i];
    if (tf2::getYaw(latest_pose.pose.orientation) != 0.0) {
      continue;
    }
    geometry_msgs::msg::PoseStamped & prev_pose = historical_poses_.at(i + 1);

    double detx = latest_pose.pose.position.x - prev_pose.pose.position.x;
    double dety = latest_pose.pose.position.y - prev_pose.pose.position.y;
    double yaw = atan2(dety, detx);

    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, yaw);
    latest_pose.pose.orientation.x = qtn.getX();
    latest_pose.pose.orientation.y = qtn.getY();
    latest_pose.pose.orientation.z = qtn.getZ();
    latest_pose.pose.orientation.w = qtn.getW();
  }

}

void TargetUpdater::truncatHinderPosesAndAppendCurPose()
{
  geometry_msgs::msg::PoseStamped pose_based_on_global_frame;

  if (!nav2_util::getCurrentPose(pose_based_on_global_frame, *tf_buffer_, global_frame_)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "truncat overdue poses failed to obtain current pose based on map coordinate system.");
    return;
  }

  double yaw = tf2::getYaw(pose_based_on_global_frame.pose.orientation);
  double p1_x, p1_y, p2_x, p2_y;
  p2_x = cos(yaw);
  p2_y = sin(yaw);
  while (historical_poses_.size() > 0) {

    p1_x = historical_poses_.back().pose.position.x -
      pose_based_on_global_frame.pose.position.x;
    p1_y = historical_poses_.back().pose.position.y -
      pose_based_on_global_frame.pose.position.y;
    if (p1_x * p2_x + p1_y * p2_y <= 0) {
      historical_poses_.pop_back();
    } else {
      break;
    }
  }
  if (historical_poses_.size() > 1 && poseDistanceSq(
      historical_poses_.back().pose,
      pose_based_on_global_frame.pose) < dist_sq_throttle_)
  {
    historical_poses_.pop_back();
  }
  historical_poses_.push_back(pose_based_on_global_frame);

}


std::vector<double> polyfit(
  const std::vector<double> & x,
  const std::vector<double> & y,
  int order = 3)
{
  std::vector<double> coeff;
  // Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial,
  // for exame k = 3 for cubic polynomial
  Eigen::MatrixXd T(x.size(), order + 1);
  Eigen::VectorXd V = Eigen::VectorXd::Map(&y.front(), y.size());
  Eigen::VectorXd result;

  // check to make sure inputs are correct
  assert(x.size() == y.size());

  // Populate the matrix
  for (size_t i = 0; i < x.size(); ++i) {
    for (int j = 0; j < order + 1; ++j) {
      T(i, j) = pow(x.at(i), j);
    }
  }

  // Solve for linear least square fit
  result = T.householderQr().solve(V);
  coeff.resize(order + 1);
  for (int k = 0; k < order + 1; k++) {
    coeff[k] = result[k];
  }
  return coeff;
}
nav_msgs::msg::Path spline(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  nav_msgs::msg::Path path;
  if (poses.size() < 2) {return path;}

  std::vector<double> x, y;
  for (auto p : poses) {
    x.push_back(p.pose.position.x);
    y.push_back(p.pose.position.y);
  }

  double max_x = *std::max_element(x.begin(), x.end());
  double min_x = *std::min_element(x.begin(), x.end());
  std::vector<double> A = polyfit(x, y);
  geometry_msgs::msg::PoseStamped pose;
  double cx = min_x;
  while (cx <= max_x + 0.001) {
    double cy = A[0] + A[1] * cx + A[2] *
      (pow(cx, 2)) + A[3] * (pow(cx, 3));
    pose.pose.position.x = cx;
    pose.pose.position.y = cy;
    path.poses.push_back(pose);
    cx += 0.05;
  }
  return path;
}
}  // namespace mcr_tracking_components

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mcr_tracking_components::TargetUpdater>("TargetUpdater");
}
