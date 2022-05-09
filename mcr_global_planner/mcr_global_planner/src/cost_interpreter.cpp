#include "mcr_global_planner/cost_interpreter.h"
#include "nav2_util/node_utils.hpp"

namespace mcr_global_planner
{

void CostInterpreter::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  name_ = name;
  costmap_ = std::shared_ptr<nav2_costmap_2d::Costmap2D>(costmap->getCostmap());
  int neutral_cost;

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".neutral_cost", rclcpp::ParameterValue(
      50));
  node->get_parameter(name_ + ".neutral_cost", neutral_cost);

  if (neutral_cost < 0 || neutral_cost > std::numeric_limits<unsigned char>::max()) {
    throw std::invalid_argument(
            "neutral_cost (" + std::to_string(
              neutral_cost) + ") must be a valid unsigned char!");
  }

  float scale;
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".scale", rclcpp::ParameterValue(3.0));
  node->get_parameter(name_ + ".scale", scale);

  UnknownInterpretation mode = UnknownInterpretation::EXPENSIVE;

  if (node->has_parameter("unknown_interpretation")) {
    if (node->has_parameter("allow_unknown")) {
      RCLCPP_ERROR(
        node->get_logger(),
        "allow_unknown can't be specified at the same time as unknown_interpretation.");
      RCLCPP_ERROR(node->get_logger(), "Using the value of unknown_interpretation.");
    }
    std::string unknown_str;
    node->get_parameter("unkonw_interpretation", unknown_str);

    if (unknown_str == "lethal") {
      mode = UnknownInterpretation::LETHAL;
    } else if (unknown_str == "expensive") {
      mode = UnknownInterpretation::EXPENSIVE;
    } else if (unknown_str == "free") {
      mode = UnknownInterpretation::FREE;
    } else {
      RCLCPP_ERROR(
        node->get_logger(), "Unknown value for unknown_interpretation '%s'. "
        "Using expensive instead.", unknown_str.c_str());
      mode = UnknownInterpretation::EXPENSIVE;
    }
  }

  setConfiguration(static_cast<unsigned char>(neutral_cost), scale, mode);
}

void CostInterpreter::setConfiguration(
  const unsigned char neutral_cost, const float scale,
  const UnknownInterpretation mode)
{
  neutral_cost_ = neutral_cost;
  for (unsigned int i = 0; i < cached_costs_.size(); i++) {
    if (i == nav2_costmap_2d::NO_INFORMATION) {
      float c;
      switch (mode) {
        case UnknownInterpretation::LETHAL:
          c = LETHAL_COST_F;
          break;
        case UnknownInterpretation::EXPENSIVE:
          c = LETHAL_COST_F - 1.0;
          break;
        default: // case FREE:
          c = neutral_cost_;
          break;
      }
      cached_costs_[i] = c;
    } else if (i <= LETHAL_COST - 1) {
      float c = i * scale + neutral_cost_;
      cached_costs_[i] = std::min(c, LETHAL_COST_F - 1.0f);
    } else {
      cached_costs_[i] = LETHAL_COST_F;
    }
  }
}
}
