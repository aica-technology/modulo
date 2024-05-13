#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace modulo_utils {

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
 * @details This functions removes all characters different from a-z, A-Z, 0-9, and _ from a string and transforms
 * uppercase letters into lowercase letters. Additionally, it removes all leading numbers and underscores, such that the
 * resulting string starts with a letter a-z.
 * @param topic_name The input string
 * @return the sanitized string
 */
[[maybe_unused]] static std::string parse_topic_name(const std::string& topic_name) {
  std::string output;
  for (char c : topic_name) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) {
      output.insert(output.end(), std::tolower(c));
    } else if (!output.empty() && ((c >= '0' && c <= '9') || c == '_')) {
      output.insert(output.end(), c);
    }
  }
  return output;
}

}// namespace modulo_utils
