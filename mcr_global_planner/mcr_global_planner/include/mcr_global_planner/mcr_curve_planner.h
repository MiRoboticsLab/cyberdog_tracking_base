#ifndef _MCR_CURVE_PLANNER_H_
#define _MCR_CURVE_PLANNER_H_

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <mcr_global_planner/cost_interpreter.h>
#include <mcr_global_planner/potential_calculator.h>
#include <mcr_global_planner/traceback.h>
#include <nav_2d_msgs/msg/pose2_d_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>

namespace mcr_global_planner{

namespace ob = ompl::base;

class MCRCurvePlanner : public nav2_core::GlobalPlanner {
public:
    MCRCurvePlanner(){};
    ~MCRCurvePlanner(){};

    /**
     * @brief Configuring plugin
     * @param parent Lifecycle node pointer
     * @param name Name of plugin map
     * @param tf Shared ptr of TF2 buffer
     * @param costmap_ros Costmap2DROS object
     */
   void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

    /**
     * @brief Cleanup lifecycle node
     */
    void cleanup() override;

    /**
     * @brief Activate lifecycle node
     */
    void activate() override;

    /**
     * @brief Deactivate lifecycle node
     */
    void deactivate() override;


    /**
     * @brief Creating a plan from start and goal poses
     * @param start Start pose
     * @param goal Goal pose
     * @return nav_msgs::Path of the generated path
     */
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;

    virtual bool isPlanValid(const nav_msgs::msg::Path &path) const;

protected:
    nav2_util::LifecycleNode::SharedPtr node_;
    std::string global_frame_, name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;

    std::string curve_type_;
    ob::StateSpacePtr space_;
    double turning_radius_;
};
}

#endif