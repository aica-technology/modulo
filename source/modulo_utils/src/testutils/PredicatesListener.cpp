#include "modulo_utils/testutils/PredicatesListener.hpp"

namespace modulo_utils::testutils {

PredicatesListener::PredicatesListener(
    const std::string& node, const std::vector<std::string>& predicates, const rclcpp::NodeOptions& node_options)
    : rclcpp::Node("predicates_listener", node_options) {
  this->received_future_ = this->received_.get_future();
  for (const auto& predicate : predicates) {
    this->predicates_.insert_or_assign(predicate, false);
  }
  this->subscription_ = this->create_subscription<modulo_interfaces::msg::PredicateCollection>(
      "/predicates", 10, [this, node](const std::shared_ptr<modulo_interfaces::msg::PredicateCollection> message) {
        if (message->node == node) {
          for (const auto& predicate_msg : message->predicates) {
            if (this->predicates_.find(predicate_msg.predicate) != this->predicates_.end()) {
              this->predicates_.at(predicate_msg.predicate) = predicate_msg.value;
              if (predicate_msg.value) {
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
