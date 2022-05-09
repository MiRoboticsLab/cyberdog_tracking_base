#ifndef _POLYNOMIAL_INTERPOLATION_H_
#define _POLYNOMIAL_INTERPOLATION_H_

#include <mcr_global_planner/spliner.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

namespace mcr_planner_plugins
{

class PolynomialInterpolation : public mcr_global_planner::Spliner
{
public:
  virtual nav_msgs::msg::Path spline(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
  override;
  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap
  ) override;

protected:
  std::vector<geometry_msgs::msg::PoseStamped>&& interpolation(
    const std::vector<geometry_msgs::msg::PoseStamped> & control_points);
  void calculateT(double t0, double t1);

private:
  double det_ = 0.05;
  double max_speed_ = 1.0;
  double dist_scale_ = 1.0;
  nav2_util::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  Eigen::Matrix<double, 6, 6> T;
  Eigen::Matrix<double, 6, 1> A, B;
  Eigen::Matrix<double, 6, 1> X;
  Eigen::Matrix<double, 6, 1> Y;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif
