#include <mcr_planner_plugins/polynomial_fitting.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <algorithm>
#include <Eigen/QR>

namespace mcr_planner_plugins
{

void PolynomialFitting::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap
)
{
  node_ = node;
  name_ = name;
  costmap_ros_ = costmap;
  det_ = 0.05;

}

nav_msgs::msg::Path PolynomialFitting::spline(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
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

std::vector<double> PolynomialFitting::polyfit(
  const std::vector<double> & x,
  const std::vector<double> & y,
  int order)
{
  std::vector<double> coeff;
  // Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
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


}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mcr_planner_plugins::PolynomialFitting, mcr_global_planner::Spliner)
