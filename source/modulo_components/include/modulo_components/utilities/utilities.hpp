#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

/**
 * @namespace modulo_components::utilities
 * @brief Modulo component utilities
 */
namespace modulo_components::utilities {

/**
 * @brief Parse a string argument value from an argument list given a pattern prefix.
 * @param args a vector of string arguments
 * @param pattern the prefix pattern to find and strip
 * @param result the default argument value that is overwritten by reference if the given pattern is found
 * @return the value of the resultant string
 */
[[maybe_unused]] static std::string
parse_string_argument(const std::vector<std::string>& args, const std::string& pattern, std::string& result) {
  for (const auto& arg : args) {
    std::string::size_type index = arg.find(pattern);
    if (index != std::string::npos) {
      result = arg;
      result.erase(index, pattern.length());
      break;
    }
  }
  return result;
}

/**
 * @brief Parse a string node name from NodeOptions arguments.
 * @param options the NodeOptions structure to parse
 * @param fallback the default name if the NodeOptions structure cannot be parsed
 * @return the parsed node name or the fallback name
 */
[[maybe_unused]]  static std::string
parse_node_name(const rclcpp::NodeOptions& options, const std::string& fallback = "") {
  std::string node_name(fallback);
  const std::string pattern("__node:=");
  return parse_string_argument(options.arguments(), pattern, node_name);
}

/**
 * @brief Parse a string topic name from a user-provided input.
 * @details This functions removes all characters different from
 * a-z, A-Z, 0-9, and _ from a string.
 * @param topic_name The input string
 * @return The sanitized string
 */
[[maybe_unused]] static std::string parse_topic_name(const std::string& topic_name) {
  std::string output;
  for (char c : topic_name) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) {
      output.insert(output.end(), std::tolower(c));
    } else if (!output.empty() && ((c >= '0' && c <= '9') || c == '_')) {
      output.insert(output.end(), std::tolower(c));
    }
  }
  return output;
}

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
