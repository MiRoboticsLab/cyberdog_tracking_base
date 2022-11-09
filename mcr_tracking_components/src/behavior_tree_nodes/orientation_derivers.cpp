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


void FittingOrientationDeriver::initialize(const rclcpp::Node::SharedPtr node,
                                        const std::string &global_frame, 
                                        const std::shared_ptr<tf2_ros::Buffer> tf_buffer){
  OrientationDeriver::initialize(node, global_frame, tf_buffer);
  plan_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("plan_derived", 1);

}

geometry_msgs::msg::PoseStamped 
FittingOrientationDeriver::deriveOrientation(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
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

    msg_with_orientation.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);

    if (!nav2_util::transformPoseInTargetFrame(
        msg_with_orientation, msg_with_orientation1, *tf_buffer_,
        msg->header.frame_id))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Faild transform target pose to %s", msg->header.frame_id.c_str());
      throw nav2_core::TFException("Transformed error in target updater node");
    }
  }

  msg_with_orientation1.pose.position = msg->pose.position;
  return msg_with_orientation1;
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

nav_msgs::msg::Path FittingOrientationDeriver::spline(
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




void KalmanOrientationDeriver::initialize(const rclcpp::Node::SharedPtr node,
                                        const std::string &global_frame, 
                                        const std::shared_ptr<tf2_ros::Buffer> tf_buffer){
  OrientationDeriver::initialize(node, global_frame, tf_buffer);
  x_ << 0.0,0.0,0.0,0.0;            // 4x1  initial state.
  U_ << 0.0,0.0,0.0,0.0;            // 4x1  external motion.
  P_ << 0.1, 0.0, 0.0, 0.0,
        0.0, 0.1, 0.0, 0.0,
        0.0, 0.0, 0.1, 0.0,
        0.0, 0.0, 0.0, 0.1;        // initial uncertainty.
  F_ << 1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 1.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;         // next state function.
  I_ = Eigen::Matrix4d::Identity(); // identity matrix.
  H_ << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0;         // measurement function.
  R_ << 1.0, 0.0, 0.0, 1.0;          // measurement uncertainty.

}

geometry_msgs::msg::PoseStamped 
KalmanOrientationDeriver::deriveOrientation(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
  geometry_msgs::msg::PoseStamped msg_with_orientation, msg_with_orientation1;
  //该模式下的跟随位姿对应在全局坐标系上的位姿
  if (!nav2_util::transformPoseInTargetFrame(
      *msg, msg_with_orientation, *tf_buffer_,
      global_frame_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Faild transform target pose to %s", global_frame_.c_str());
    throw nav2_core::TFException("Transformed error in target updater node");
  }

  if(historical_raw_poses_.size() > 1){
    rclcpp::Time t1 = msg->header.stamp;
    rclcpp::Time t0 = historical_raw_poses_[0].header.stamp;
    double dt = (t1 - t0).seconds();
    vx_ = (msg_with_orientation.pose.position.x - historical_raw_poses_[0].pose.position.x) / dt;
    vy_ = (msg_with_orientation.pose.position.y - historical_raw_poses_[0].pose.position.y) / dt;
  }else{
    historical_raw_poses_.push_front(msg_with_orientation);
    return *msg;
  }

  //push pose when robot has moved a little.
  if (sqrt(poseDistanceSq(historical_raw_poses_.front().pose, msg_with_orientation.pose)) > 0.3) {
    historical_raw_poses_.push_front(msg_with_orientation);
  }
  // deal with pose's orientation
  while (historical_raw_poses_.size() > 5) {
    historical_raw_poses_.pop_back();
  }
  double theta = 0.0;

  double x = historical_raw_poses_.back().pose.position.x;
  double y = historical_raw_poses_.back().pose.position.y;
  
  x_ << x, y, 0.0, 0.0;
  for(auto it = historical_raw_poses_.rbegin() + 1; it != historical_raw_poses_.rend(); it++){
    // measurement update
    Eigen::Matrix<double, 2, 1> z;
    z << it->pose.position.x, it->pose.position.y;
    Eigen::Matrix<double, 2, 1> y = z - (H_ * x_);
    Eigen::Matrix<double, 2, 2> S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + (K * y);
    P_ = (I_ - (K * H_)) * P_;
    // prediction
    x_ = (F_ * x_) + U_;
    P_ = F_ * P_ * F_.transpose();
  }

  theta = atan2(x_(3, 0), x_(2, 0));

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

PLUGINLIB_EXPORT_CLASS(
  mcr_tracking_components::FittingOrientationDeriver,
  mcr_tracking_components::OrientationDeriver)  

PLUGINLIB_EXPORT_CLASS(
  mcr_tracking_components::KalmanOrientationDeriver,
  mcr_tracking_components::OrientationDeriver)  
