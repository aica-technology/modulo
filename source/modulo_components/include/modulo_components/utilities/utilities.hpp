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
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_') {
      if (!(c == '_' && output.empty())) {
        output.insert(output.end(), std::tolower(c));
      }
    }
  }
  return output;
}

/**
 * @brief Generate the topic name for a predicate from component name and predicate name.
 * @param component_name The name of the component which the predicate belongs to
 * @param predicate_name The name of the predicate
 * @return The generated predicate topic as /predicates/component_name/predicate_name
 */
[[maybe_unused]] static std::string
generate_predicate_topic(const std::string& component_name, const std::string& predicate_name) {
  return "/predicates/" + component_name + "/" + predicate_name;
}
}// namespace modulo_components::utilities
