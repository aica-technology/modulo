#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

/**
 * @namespace modulo_utils::parsing
 * @brief Modulo parsing helpers.
 */
namespace modulo_utils::parsing {

/**
 * @brief This function returns a default validation warning for topic names.
 * @param name The pre-validation signal name
 * @param type One of input|output
 * @return The validation warning
 * @see modulo_utils::parsing::parse_topic_name
*/
[[maybe_unused]] static const std::string topic_validation_warning(const std::string& name, const std::string& type) {
  return "The parsed signal name for " + type + " '" + name
      + "' is empty. Provide a string with valid characters for the signal name ([a-z0-9_]).";
}

/**
 * @brief Parse a string node name from NodeOptions arguments.
 * @param options The NodeOptions structure to parse
 * @param fallback The default name if the NodeOptions structure cannot be parsed
 * @return the parsed node name or the fallback name
 */
[[maybe_unused]] static std::string
parse_node_name(const rclcpp::NodeOptions& options, const std::string& fallback = "") {
  std::string node_name = fallback;
  const std::string pattern("__node:=");
  for (const auto& arg : options.arguments()) {
    std::string::size_type index = arg.find(pattern);
    if (index != std::string::npos) {
      node_name = arg;
      node_name.erase(index, pattern.length());
      break;
    }
  }
  return node_name;
}

/**
 * @brief Parse a string topic name from a user-provided input.
 * @details This functions removes all characters different from a-z, 0-9, and _ from a string. Additionally, it removes
 * all leading numbers and underscores, such that the resulting string starts with a letter a-z.
 * @param topic_name The input string
 * @return the sanitized string
 */
[[maybe_unused]] static std::string parse_topic_name(const std::string& topic_name) {
  std::string output;
  for (char c : topic_name) {
    if (c >= 'a' && c <= 'z') {
      output.insert(output.end(), c);
    } else if (!output.empty() && ((c >= '0' && c <= '9') || c == '_')) {
      output.insert(output.end(), c);
    }
  }
  return output;
}

}// namespace modulo_utils::parsing
