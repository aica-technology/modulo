#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace modulo_utils::testutils {

using namespace std::chrono_literals;

class PredicatesListener : public rclcpp::Node {
public:
  PredicatesListener(
      const rclcpp::NodeOptions& node_options, const std::string& ns, const std::vector<std::string>& predicates
  );

  void reset_future();

  const std::shared_future<void>& get_predicate_future() const;

  const std::map<std::string, bool>& get_predicate_values() const;

private:
  std::map<std::string, std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>>> subscriptions_;
  std::map<std::string, bool> predicates_;
  std::shared_future<void> received_future_;
  std::promise<void> received_;
};
}// namespace modulo_utils::testutils
