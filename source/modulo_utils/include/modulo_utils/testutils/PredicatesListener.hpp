#pragma once

#include <future>
#include <map>

#include <modulo_interfaces/msg/predicate_collection.hpp>
#include <rclcpp/rclcpp.hpp>

namespace modulo_utils::testutils {

using namespace std::chrono_literals;

class PredicatesListener : public rclcpp::Node {
public:
  PredicatesListener(
      const std::string& node, const std::vector<std::string>& predicates,
      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  void reset_future();

  [[nodiscard]] const std::shared_future<void>& get_predicate_future() const;

  [[nodiscard]] const std::map<std::string, bool>& get_predicate_values() const;

private:
  std::shared_ptr<rclcpp::Subscription<modulo_interfaces::msg::PredicateCollection>> subscription_;
  std::map<std::string, bool> predicates_;
  std::shared_future<void> received_future_;
  std::promise<void> received_;
};
}// namespace modulo_utils::testutils
