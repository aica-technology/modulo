#include "modulo_utils/testutils/PredicatesListener.hpp"

namespace modulo_utils::testutils {

PredicatesListener::PredicatesListener(
    const std::string& component, const std::vector<std::string>& predicates, const rclcpp::NodeOptions& node_options
) : rclcpp::Node("predicates_listener", node_options) {
  this->received_future_ = this->received_.get_future();
  for (const auto& predicate : predicates) {
    this->predicates_.insert_or_assign(predicate, false);
  }
  this->subscription_ = this->create_subscription<modulo_component_interfaces::msg::Predicate>(
      "/predicates", 10, [this, component](const std::shared_ptr<modulo_component_interfaces::msg::Predicate> message) {
        if (message->component == component) {
          for (auto& predicate : this->predicates_) {
            if (message->predicate == predicate.first) {
              predicate.second = message->value;
              if (message->value) {
                this->received_.set_value();
              }
            }
          }
        }
      });
}

void PredicatesListener::reset_future() {
  this->received_ = std::promise<void>();
  this->received_future_ = this->received_.get_future();
}

const std::shared_future<void>& PredicatesListener::get_predicate_future() const {
  return this->received_future_;
}

const std::map<std::string, bool>& PredicatesListener::get_predicate_values() const {
  return this->predicates_;
}
}// namespace modulo_utils::testutils