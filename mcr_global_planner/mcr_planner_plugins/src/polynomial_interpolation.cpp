#include <mcr_planner_plugins/polynomial_interpolation.h>
#include <nav2_util/node_utils.hpp>

namespace mcr_planner_plugins
{


void PolynomialInterpolation::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap
)
{
  node_ = node;
  name_ = name;
  costmap_ros_ = costmap;

  double max_speed_ = 1.0;
  double dist_scale_ = 1.0;
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_speed", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".max_speed", max_speed_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".dist_scale", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".dist_scale", dist_scale_);
}

void PolynomialInterpolation::calculateT(double t0, double t1)
{
  T << pow(t0, 5), pow(t0, 4), pow(t0, 3), pow(t0, 2), t0, 1,
    5 * pow(t0, 4), 4 * pow(t0, 3), 3 * pow(t0, 2), 2 * t0, 1, 0,
    20 * pow(t0, 3), 12 * pow(t0, 2), 6 * t0, 2, 0, 0,
    pow(t1, 5), pow(t1, 4), pow(t1, 3), pow(t1, 2), t1, 1,
    5 * pow(t1, 4), 4 * pow(t1, 3), 3 * pow(t1, 2), 2 * t1, 1, 0,
    20 * pow(t1, 3), 12 * pow(t1, 2), 6 * t1, 2, 0, 0;
}

nav_msgs::msg::Path PolynomialInterpolation::spline(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  if (poses.size() < 2) {return path;}

  std::vector<geometry_msgs::msg::PoseStamped> control_points;
  for (size_t i = 0; i < poses.size() - 1; i++) {
    control_points.clear();
    control_points.push_back(poses[i]);
    control_points.push_back(poses[i + 1]);

    std::vector<geometry_msgs::msg::PoseStamped>&& poses_t = interpolation(control_points);
    path.poses.insert(path.poses.end(), poses_t.begin(), poses_t.end());
  }
  return path;
}


std::vector<geometry_msgs::msg::PoseStamped>&& PolynomialInterpolation::interpolation(
  const std::vector<geometry_msgs::msg::PoseStamped>&control_points)
{
  double max_speed = max_speed_;
  static std::vector<geometry_msgs::msg::PoseStamped> path;
  path.clear();
  if (control_points.size() < 2) {
    return std::move(path);
  }

  double x0 = control_points.front().pose.position.x, y0 = control_points.front().pose.position.y;
  double yaw = tf2::getYaw(control_points.front().pose.orientation);
  double x0_ = max_speed * cos(yaw), y0_ = max_speed * sin(yaw), x0__ = 0.0, y0__ = 0.0;
  double x1 = control_points.back().pose.position.x, y1 = control_points.back().pose.position.y;
  yaw = tf2::getYaw(control_points.back().pose.orientation);
  double x1_ = max_speed * cos(yaw), y1_ = max_speed * sin(yaw), x1__ = 0.0, y1__ = 0.0;

  double t0 = 0.0;
  double t1 = hypot(x1 - x0, y1 - y0) / max_speed * dist_scale_;

  calculateT(t0, t1);

  X << x0, x0_, x0__, x1, x1_, x1__;
  Y << y0, y0_, y0__, y1, y1_, y1__;

  A = T.lu().solve(X);
  B = T.lu().solve(Y);

  double t = t0;
  double x, y;
  geometry_msgs::msg::PoseStamped pose;
  path.push_back(control_points.front());
  while (t < t1) {
    t += det_;
    x = A(0, 0) * pow(t, 5) + A(1, 0) * pow(t, 4) + A(2, 0) * pow(t, 3) + A(3, 0) * pow(t, 2) +
      A(4, 0) * t + A(5, 0);
    y = B(0, 0) * pow(t, 5) + B(1, 0) * pow(t, 4) + B(2, 0) * pow(t, 3) + B(3, 0) * pow(t, 2) +
      B(4, 0) * t + B(5, 0);
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    path.push_back(pose);
  }
  path.push_back(control_points.back());
  return std::move(path);
}


}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mcr_planner_plugins::PolynomialInterpolation, mcr_global_planner::Spliner)
