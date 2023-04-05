#include "modulo_utils/testutils/PredicatesListener.hpp"

namespace modulo_utils::testutils {

PredicatesListener::PredicatesListener(
    const rclcpp::NodeOptions& node_options, const std::string& ns, const std::vector<std::string>& predicates
) : rclcpp::Node("predicates_listener", node_options) {
  for (const auto& predicate : predicates) {
    this->predicates_.insert_or_assign(predicate, false);
    auto subscription = this->create_subscription<std_msgs::msg::Bool>(
        "/predicates/" + ns + "/" + predicate, 10,
        [this, predicate](const std::shared_ptr<std_msgs::msg::Bool> message) {
          this->predicates_.at(predicate) = message->data;
        });
    this->subscriptions_.insert_or_assign(predicate, subscription);
  }
  this->received_future_ = this->received_.get_future();
  this->create_wall_timer(
      100ms, [this]() {
        for (auto [first, second] : this->predicates_) {
          if (second) {
            this->received_.set_value();
            break;
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