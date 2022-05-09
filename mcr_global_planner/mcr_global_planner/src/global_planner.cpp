#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/lifecycle_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "mcr_global_planner/mcr_global_planner.h"


class PlannerWrapper : public nav2_util::LifecycleNode
{
public:
  PlannerWrapper()
  : nav2_util::LifecycleNode("mcr_planner", "", true),
    gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
    costmap_(nullptr)
  {
    start_.header.frame_id = "map";
    RCLCPP_INFO(get_logger(), "Creating");

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&PlannerWrapper::onGoalPoseReceived, this, std::placeholders::_1));

    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&PlannerWrapper::onInitialPoseReceived, this, std::placeholders::_1));


    // Setup the global costmap
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", std::string{get_namespace()}, "global_costmap");

    // Launch a thread to run the costmap node
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
  }

  ~PlannerWrapper()
  {
    RCLCPP_INFO(get_logger(), "Destroying");
    costmap_thread_.reset();
  }

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    costmap_ros_->on_configure(state);
    costmap_ = costmap_ros_->getCostmap();

    RCLCPP_DEBUG(
      get_logger(), "Costmap size: %d,%d",
      costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

    tf_ = costmap_ros_->getTfBuffer();
    auto node = shared_from_this();
    std::string planner_name, plugin_class;
    nav2_util::declare_parameter_if_not_declared(
      node, "planner_name",
      rclcpp::ParameterValue("curve_planner"));
    get_parameter("planner_name", planner_name);
    plugin_class = nav2_util::get_plugin_type_param(node, planner_name);

    try {
      planner_ = gp_loader_.createUniqueInstance(plugin_class);
      RCLCPP_INFO(
        get_logger(), "Created global planner plugin of type %s", plugin_class.c_str());
      planner_->configure(node, planner_name, tf_, costmap_ros_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create global planner. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }

    // Initialize pubs & subs
    plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);
    return nav2_util::CallbackReturn::SUCCESS;
  }
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Activating");

    plan_publisher_->on_activate();
    costmap_ros_->on_activate(state);
    planner_->activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating");

    plan_publisher_->on_deactivate();
    costmap_ros_->on_deactivate(state);
    planner_->activate();

    return nav2_util::CallbackReturn::SUCCESS;
  }
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");

    plan_publisher_.reset();
    tf_.reset();
    costmap_ros_->on_cleanup(state);
    planner_->cleanup();
    costmap_ = nullptr;

    return nav2_util::CallbackReturn::SUCCESS;
  }
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void publishPlan(const nav_msgs::msg::Path & path)
  {
    auto msg = std::make_unique<nav_msgs::msg::Path>(path);
    if (
      plan_publisher_->is_activated() &&
      this->count_subscribers(plan_publisher_->get_topic_name()) > 0)
    {
      plan_publisher_->publish(std::move(msg));
    }
  }

  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    try {
      nav_msgs::msg::Path path =
        planner_->createPlan(start_, *pose);
      publishPlan(path);
    } catch (...) {
      RCLCPP_WARN(
        get_logger(), "Failed to get path for the target (%f, %f, %f)",
        pose->pose.position.x, pose->pose.position.y,
        tf2::getYaw(pose->pose.orientation));
    }
  }

  void onInitialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
  {
    start_.header = pose->header;
    start_.pose = pose->pose.pose;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  nav2_costmap_2d::Costmap2D * costmap_;
  nav2_core::GlobalPlanner::Ptr planner_;
  geometry_msgs::msg::PoseStamped start_;
  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerWrapper>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
