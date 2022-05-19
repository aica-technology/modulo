#pragma once

#include "modulo_new_core/communication/SubscriptionInterface.hpp"

namespace modulo_new_core::communication {

template<typename MsgT>
class SubscriptionHandler : public SubscriptionInterface {
public:
  explicit SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair = nullptr);

  [[nodiscard]] std::shared_ptr<rclcpp::Subscription<MsgT>> get_subscription() const;

  void set_subscription(const std::shared_ptr<rclcpp::Subscription<MsgT>>& subscription);

  std::function<void(const std::shared_ptr<MsgT>)> get_callback();

  std::shared_ptr<SubscriptionInterface>
  create_subscription_interface(const std::shared_ptr<rclcpp::Subscription<MsgT>>& subscription);

private:
  std::shared_ptr<rclcpp::Subscription<MsgT>> subscription_;
};

template<typename MsgT>
std::shared_ptr<rclcpp::Subscription<MsgT>> SubscriptionHandler<MsgT>::get_subscription() const {
  return this->subscription_;
}

template<typename MsgT>
void SubscriptionHandler<MsgT>::set_subscription(const std::shared_ptr<rclcpp::Subscription<MsgT>>& subscription) {
  if (subscription == nullptr) {
    throw exceptions::NullPointerException("Provide a valid pointer");
  }
  this->subscription_ = subscription;
}

template<typename MsgT>
std::shared_ptr<SubscriptionInterface> SubscriptionHandler<MsgT>::create_subscription_interface(
    const std::shared_ptr<rclcpp::Subscription<MsgT>>& subscription
) {
  this->set_subscription(subscription);
  return std::shared_ptr<SubscriptionInterface>(this->shared_from_this());
}

}// namespace modulo_new_core::communication