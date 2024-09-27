#pragma once

#include <rclcpp/rclcpp.hpp>

#include "modulo_core/communication/MessagePair.hpp"
#include "modulo_core/communication/PublisherHandler.hpp"
#include "modulo_core/communication/SubscriptionHandler.hpp"

using namespace std::chrono_literals;

namespace modulo_core::communication {

template<typename MsgT>
class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher(const std::string& topic_name, std::shared_ptr<MessagePairInterface> message_pair)
      : Node("minimal_publisher") {
    auto publisher = this->create_publisher<MsgT>(topic_name, 10);
    this->publisher_interface_ =
        std::make_shared<PublisherHandler<rclcpp::Publisher<MsgT>, MsgT>>(PublisherType::PUBLISHER, publisher)
            ->create_publisher_interface(message_pair);
    timer_ = this->create_wall_timer(10ms, [this]() { this->publisher_interface_->publish(); });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<PublisherInterface> publisher_interface_;
};

template<typename MsgT>
class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber(const std::string& topic_name, std::shared_ptr<MessagePairInterface> message_pair)
      : Node("minimal_subscriber") {
    this->received_future = this->received_.get_future();
    this->subscription_interface_ = std::make_shared<SubscriptionHandler<MsgT>>(message_pair);
    auto handler = this->subscription_interface_->template get_handler<MsgT>();
    handler->set_user_callback([this] { this->received_.set_value(); });
    auto subscription = this->create_subscription<MsgT>(topic_name, 10, handler->get_callback());
    this->subscription_interface_ =
        this->subscription_interface_->template get_handler<MsgT>()->create_subscription_interface(subscription);
  }

  std::shared_future<void> received_future;

private:
  std::promise<void> received_;
  std::shared_ptr<SubscriptionInterface> subscription_interface_;
};
}// namespace modulo_core::communication
