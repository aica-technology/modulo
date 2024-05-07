#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

/**
 * @namespace modulo_components::utilities
 * @brief Modulo component utilities
 */
namespace modulo_components::utilities {

/**
 * @brief Modify parameter overrides to handle the rate and period parameters
 * @details This function checks for existence of the rate and period parameter in the parameter overrides
 * and modifies them to correspond to the same value. If both the rate and the period parameter exist, the period will
 * be set from the rate.
 * @param options The node options passed to the component constructor
 * @return The same node options with modified parameter overrides
*/
[[maybe_unused]] static rclcpp::NodeOptions modify_parameter_overrides(const rclcpp::NodeOptions& options) {
  auto modified = options;
  rclcpp::Parameter rate;
  rclcpp::Parameter period;
  std::vector<rclcpp::Parameter> parameters;
  for (const auto& parameter : options.parameter_overrides()) {
    if (parameter.get_name() == "rate") {
      rate = parameter;
    } else if (parameter.get_name() == "period") {
      period = parameter;
    } else {
      parameters.push_back(parameter);
    }
  }
  if (rate.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    period = rclcpp::Parameter("period", 1.0 / rate.as_int());
  } else if (period.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    rate = rclcpp::Parameter("rate", static_cast<int>(1.0 / period.as_double()));
  } else {
    modified.parameter_overrides() = parameters;
    return modified;
  }
  parameters.push_back(rate);
  parameters.push_back(period);
  modified.parameter_overrides() = parameters;
  return modified;
}
}// namespace modulo_components::utilities
