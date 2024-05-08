#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

/**
 * @namespace modulo_controllers::utilities
 * @brief Modulo controllers utilities
 */
namespace modulo_controllers::utilities {

typedef std::variant<bool, std::function<bool(void)>> PredicateVariant;

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
}// namespace modulo_controllers::utilities
