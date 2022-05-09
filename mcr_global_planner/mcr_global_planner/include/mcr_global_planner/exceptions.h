#ifndef _EXCEPTION_H_
#define _EXCEPTION_H_
#include <string>
#include "nav2_core/exceptions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace mcr_global_planner {

  inline std::string poseToString(const geometry_msgs::msg::PoseStamped & pose)
  {
    return "(" + std::to_string(pose.pose.position.x) + ", " +
           std::to_string(pose.pose.position.y) + ", " +
           std::to_string(tf2::getYaw(pose.pose.orientation)) + " : " + pose.header.frame_id + ")";
  }

/**
 * @class GlobalPlannerException
 * @brief General container for exceptions thrown from the Global Planner
 */
  class GlobalPlannerException: public nav2_core::PlannerException
  {
public:
    explicit GlobalPlannerException(const std::string & description)
    : nav2_core::PlannerException(description) {
    }
  };

/**
 * @class InvalidStartPoseException
 * @brief Exception thrown when there is a problem at the start location for the global planner
 */
  class InvalidStartPoseException: public GlobalPlannerException
  {
public:
    explicit InvalidStartPoseException(const std::string & description)
    : GlobalPlannerException(description) {
    }
    InvalidStartPoseException(
      const geometry_msgs::msg::PoseStamped & pose,
      const std::string & problem)
      : GlobalPlannerException("The starting pose " + poseToString(pose) + " is " + problem) {
    }
  };

/**
 * @class StartBoundsException
 * @brief Exception thrown when the start location of the global planner is out of the expected bounds
 */
  class StartBoundsException: public InvalidStartPoseException
  {
public:
    explicit StartBoundsException(const std::string & description)
    : InvalidStartPoseException(description) {
    }
    explicit StartBoundsException(const geometry_msgs::msg::PoseStamped & pose)
    : InvalidStartPoseException(pose, "out of bounds") {
    }
  };

/**
 * @class OccupiedStartException
 * @brief Exception thrown when the start location of the global planner is occupied in the costmap
 */
  class OccupiedStartException: public InvalidStartPoseException
  {
public:
    explicit OccupiedStartException(const std::string & description)
    : InvalidStartPoseException(description) {
    }
    explicit OccupiedStartException(const geometry_msgs::msg::PoseStamped & pose)
    : InvalidStartPoseException(pose, "occupied") {
    }
  };


/**
 * @class InvalidGoalPoseException
 * @brief Exception thrown when there is a problem at the goal location for the global planner
 */
  class InvalidGoalPoseException: public GlobalPlannerException
  {
public:
    explicit InvalidGoalPoseException(const std::string & description)
    : GlobalPlannerException(description) {
    }
    InvalidGoalPoseException(
      const geometry_msgs::msg::PoseStamped & pose,
      const std::string & problem)
      : GlobalPlannerException("The goal pose " + poseToString(pose) + " is " + problem) {
    }
  };


/**
 * @class GoalBoundsException
 * @brief Exception thrown when the goal location of the global planner is out of the expected bounds
 */
  class GoalBoundsException: public InvalidGoalPoseException
  {
public:
    explicit GoalBoundsException(const std::string & description)
    : InvalidGoalPoseException(description) {
    }
    explicit GoalBoundsException(const geometry_msgs::msg::PoseStamped & pose)
    : InvalidGoalPoseException(pose, "out of bounds") {
    }
  };

/**
 * @class OccupiedGoalException
 * @brief Exception thrown when the goal location of the global planner is occupied in the costmap
 */

  class OccupiedGoalException: public InvalidGoalPoseException
  {
public:
    explicit OccupiedGoalException(const std::string & description)
    : InvalidGoalPoseException(description) {
    }
    explicit OccupiedGoalException(const geometry_msgs::msg::PoseStamped & pose)
    : InvalidGoalPoseException(pose, "occupied") {
    }
  };

/**
 * @class NoGlobalPathException
 * @brief Exception thrown when the global planner cannot find a path from the start to the goal
 */
  class NoGlobalPathException: public GlobalPlannerException
  {
public:
    explicit NoGlobalPathException(const std::string & description)
    : GlobalPlannerException(description) {
    }
    NoGlobalPathException() : GlobalPlannerException("No global path found.") {
    }
  };

}
#endif
