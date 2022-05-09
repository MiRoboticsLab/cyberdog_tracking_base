#include <mcr_planner_plugins/b_spliner.h>
#include <Eigen/Dense>

namespace mcr_planner_plugins
{

void BSpliner::initialize(
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

double BSpliner::deboor_cox(int i, int k, double u)
{
  double bik_u;
  if (k == 0) {
    if (u >= knots_vector_[i] && u < knots_vector_[i + 1]) {
      bik_u = 1;
    } else {
      bik_u = 0;
    }
  } else {
    double l1 = knots_vector_[i + k] - knots_vector_[i];
    double l2 = knots_vector_[i + k + 1] - knots_vector_[i + 1];
    if (l1 == 0) {
      l1 = 1;
    }
    if (l2 == 0) {
      l2 = 1;
    }

    bik_u = (u - knots_vector_[i]) / l1 * deboor_cox(i, k - 1, u) +
      (knots_vector_[i + k + 1] - u) / l2 * deboor_cox(i + 1, k - 1, u);
  }
  return bik_u;
}


nav_msgs::msg::Path BSpliner::spline(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  int k = k_;

  if (poses.size() < 2) {return path;}

  int n = poses.size() - 1;
  if(n < 4){
    k = n;
  }
  knots_vector_.resize(n + k + 1);

  std::vector<double> Bik;
  Bik.resize(n + 1);

  geometry_msgs::msg::PoseStamped pose;
  if (flag_ == 1) {
    double x = 1.0 / (knots_vector_.size() - 1);
    knots_vector_[0] = 0.0;
    for (int i = 1; i < knots_vector_.size(); ++i) {
      knots_vector_[i] = knots_vector_[i - 1] + x;
    }
    for (double u = (k - 1.0) / (n + k + 1); u <= (n + 2.0) / (n + k + 1); u += 0.01) {
      double x = 0.0, y = 0.0;
      for (int i = 0; i <= n; i++) {

        Bik[i] = deboor_cox(i, k - 1, u);

        x += Bik[i] * poses[i].pose.position.x;
        y += Bik[i] * poses[i].pose.position.y;
      }
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      path.poses.push_back(pose);
    }
  } else {
    double x = 1.0 / (knots_vector_.size() - k - k + 1);
    for (int i = 0; i < knots_vector_.size(); ++i) {
      if (i < k) {
        knots_vector_[i] = 0.0;
      }else if(k <= i && i < knots_vector_.size() - k){
        knots_vector_[i] = knots_vector_[i-1] + x;
      }else{
        knots_vector_[i] = 1.0;
      }
    }

    for (double u = 0.0; u <= 1.001; u += 0.01) {
      if(u>1.0)u=0.99999;
      double x = 0.0, y = 0.0;
      for(int i = 0; i <= n; i++){
        Bik[i] = deboor_cox(i, k - 1, u);

        x += Bik[i] * poses[i].pose.position.x;
        y += Bik[i] * poses[i].pose.position.y;
      }
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      path.poses.push_back(pose);      
    }
  }

  return path;
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mcr_planner_plugins::BSpliner, mcr_global_planner::Spliner)
