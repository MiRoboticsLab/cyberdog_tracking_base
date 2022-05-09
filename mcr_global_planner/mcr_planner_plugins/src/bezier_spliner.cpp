#include <mcr_planner_plugins/bezier_spliner.h>

namespace mcr_planner_plugins
{

void BezierSpliner::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap
)
{
  node_ = node;
  name_ = name;
  costmap_ros_ = costmap;
  det_ = 0.05;
  for (double t = 0; t <= 1.0; t += det_) {
    std::vector<double> table;
    double alp0 = -1 * t * t * t + 3 * t * t - 3 * t + 1;
    double alp1 = 3 * t * t * t - 6 * t * t + 3 * t;
    double alp2 = -3 * t * t * t + 3 * t * t;
    double alp3 = t * t * t;
    table.push_back(alp0);
    table.push_back(alp1);
    table.push_back(alp2);
    table.push_back(alp3);
    cheat_sheet_.push_back(table);
  }
}

nav_msgs::msg::Path BezierSpliner::spline(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  nav_msgs::msg::Path path;
  if (poses.size() < 2) {return path;}
  std::vector<geometry_msgs::msg::PoseStamped> control_points;
  std::vector<geometry_msgs::msg::PoseStamped>::iterator start, stop;
  int count = 0;
  for (size_t i = 0; i < poses.size() - 1; i++) {
    count += 1;
    control_points.push_back(poses[i]);
    if (count % 3 == 0) {
      geometry_msgs::msg::PoseStamped middle_pose;
      middle_pose.pose.position.x = (poses[i].pose.position.x + poses[i + 1].pose.position.x) / 2;
      middle_pose.pose.position.y = (poses[i].pose.position.y + poses[i + 1].pose.position.y) / 2;
      control_points.push_back(middle_pose);
      control_points.push_back(middle_pose);
      count += 1;
    }
  }
  control_points.push_back(poses[poses.size() - 1]);

  int i;
  int c_len = control_points.size();
  for (i = 0; i < c_len - 5; i += 4) {
    start = control_points.begin() + i;
    stop = start + 4;
    std::vector<geometry_msgs::msg::PoseStamped> cp(start, stop);
    std::vector<geometry_msgs::msg::PoseStamped> tmp = bezier(cp);
    path.poses.insert(path.poses.end(), tmp.begin(), tmp.end());
  }
  start = control_points.begin() + i;
  stop = control_points.end();
  std::vector<geometry_msgs::msg::PoseStamped> cp(start, stop);
  std::vector<geometry_msgs::msg::PoseStamped>&& tmp = bezier(cp);
  path.poses.insert(path.poses.end(), tmp.begin(), tmp.end());
  return path;
}


int factorial(int number)
{
  register int i, f = 1;
  for (i = 1; i <= number; i++) {
    f *= i;
  }
  return f;
}


std::vector<geometry_msgs::msg::PoseStamped>&& BezierSpliner::bezier(
  const std::vector<geometry_msgs::msg::PoseStamped>&control_points)
{
  static std::vector<geometry_msgs::msg::PoseStamped> path;
  path.clear();
  geometry_msgs::msg::PoseStamped pose;
  int cns = control_points.size();
  double t = 0.0;
  double x, y;
  for (size_t i = 0; i < cheat_sheet_.size(); i++) {
    if (cns == 4) {
      double alp0 = cheat_sheet_[i][0];
      double alp1 = cheat_sheet_[i][1];
      double alp2 = cheat_sheet_[i][2];
      double alp3 = cheat_sheet_[i][3];

      x = control_points[0].pose.position.x * alp0 + control_points[1].pose.position.x * alp1 +
        control_points[2].pose.position.x * alp2 + control_points[3].pose.position.x * alp3;

      y = control_points[0].pose.position.y * alp0 + control_points[1].pose.position.y * alp1 +
        control_points[2].pose.position.y * alp2 + control_points[3].pose.position.y * alp3;
    } else {
      int n = cns - 1;
      int nf = factorial(n);
      x = 0;y = 0;
      for (int k = 0; k < cns; k++) {
        int i_f = factorial(k);
        int n_i_f = factorial(n - k);
        double ratio = (nf / (i_f * n_i_f)) * pow(t, k) * pow((1 - t), n - k);

        x += ratio * control_points[k].pose.position.x;
        y += ratio * control_points[k].pose.position.y;
      }
      t = (i + 1) * det_;
    }
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    path.push_back(pose);
    t += det_;
  }

  return std::move(path);
}


}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mcr_planner_plugins::BezierSpliner, mcr_global_planner::Spliner)
