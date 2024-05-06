#pragma once

#include <future>

#include <rclcpp/rclcpp.hpp>

namespace modulo_controller_interface {

using namespace std::chrono_literals;

template<typename MsgT>
class SubscriptionNode : public rclcpp::Node {
public:
  SubscriptionNode(const std::string& topic, const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
      : rclcpp::Node("subscription_node", node_options) {
    received_future_ = received_.get_future();
    subscription_ = create_subscription<MsgT>(topic, 10, [this](const std::shared_ptr<MsgT> msg) {
      this->message = *msg;
      this->received_.set_value();
    });
  }

  [[nodiscard]] const std::shared_future<void>& get_sub_future() const { return received_future_; }

  MsgT message;

private:
  std::shared_ptr<rclcpp::Subscription<MsgT>> subscription_;
  std::shared_future<void> received_future_;
  std::promise<void> received_;
};
}// namespace modulo_controller_interface
