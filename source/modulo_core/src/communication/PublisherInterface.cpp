#include "modulo_core/communication/PublisherInterface.hpp"

#include <utility>

#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "modulo_core/communication/PublisherHandler.hpp"
#include "modulo_core/exceptions/NullPointerException.hpp"

namespace modulo_core::communication {

PublisherInterface::PublisherInterface(PublisherType type, std::shared_ptr<MessagePairInterface> message_pair) :
    type_(type), message_pair_(std::move(message_pair)) {}

void PublisherInterface::activate() {
  if (this->message_pair_ == nullptr) {
    throw exceptions::NullPointerException("Message pair is not set, cannot deduce message type");
  }
  switch (this->message_pair_->get_type()) {
    case MessageType::BOOL:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>,
                                 std_msgs::msg::Bool>()->on_activate();
      break;
    case MessageType::FLOAT64:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>,
                                 std_msgs::msg::Float64>()->on_activate();
      break;
    case MessageType::FLOAT64_MULTI_ARRAY:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>,
                                 std_msgs::msg::Float64MultiArray>()->on_activate();
      break;
    case MessageType::INT32:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>,
                                 std_msgs::msg::Int32>()->on_activate();
      break;
    case MessageType::STRING:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>,
                                 std_msgs::msg::String>()->on_activate();
      break;
    case MessageType::ENCODED_STATE:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<EncodedState>, EncodedState>()->on_activate();
      break;
  }
}

void PublisherInterface::deactivate() {
  if (this->message_pair_ == nullptr) {
    throw exceptions::NullPointerException("Message pair is not set, cannot deduce message type");
  }
  switch (this->message_pair_->get_type()) {
    case MessageType::BOOL:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>,
                                 std_msgs::msg::Bool>()->on_deactivate();
      break;
    case MessageType::FLOAT64:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>,
                                 std_msgs::msg::Float64>()->on_deactivate();
      break;
    case MessageType::FLOAT64_MULTI_ARRAY:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>,
                                 std_msgs::msg::Float64MultiArray>()->on_deactivate();
      break;
    case MessageType::INT32:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>,
                                 std_msgs::msg::Int32>()->on_deactivate();
      break;
    case MessageType::STRING:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>,
                                 std_msgs::msg::String>()->on_deactivate();
      break;
    case MessageType::ENCODED_STATE:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<EncodedState>, EncodedState>()->on_deactivate();
      break;
  }
}

void PublisherInterface::publish() {
  try {
    if (this->message_pair_ == nullptr) {
      throw exceptions::NullPointerException("Message pair is not set, nothing to publish");
    }
    switch (this->message_pair_->get_type()) {
      case MessageType::BOOL:
        this->publish(this->message_pair_->write<std_msgs::msg::Bool, bool>());
        break;
      case MessageType::FLOAT64:
        this->publish(this->message_pair_->write<std_msgs::msg::Float64, double>());
        break;
      case MessageType::FLOAT64_MULTI_ARRAY:
        this->publish(this->message_pair_->write<std_msgs::msg::Float64MultiArray, std::vector<double>>());
        break;
      case MessageType::INT32:
        this->publish(this->message_pair_->write<std_msgs::msg::Int32, int>());
        break;
      case MessageType::STRING:
        this->publish(this->message_pair_->write<std_msgs::msg::String, std::string>());
        break;
      case MessageType::ENCODED_STATE:
        if (!this->message_pair_->get_message_pair<
            EncodedState, state_representation::State
        >()->get_data()->is_empty()) {
          this->publish(this->message_pair_->write<EncodedState, state_representation::State>());
        }
        break;
    }
  } catch (const exceptions::CoreException& ex) {
    throw;
  }
}

template<typename MsgT>
void PublisherInterface::publish(const MsgT& message) {
  switch (this->get_type()) {
    case PublisherType::PUBLISHER:
      this->template get_handler<rclcpp::Publisher<MsgT>, MsgT>()->publish(message);
      break;
    case PublisherType::LIFECYCLE_PUBLISHER:
      this->template get_handler<rclcpp_lifecycle::LifecyclePublisher<MsgT>, MsgT>()->publish(message);
      break;
  }
}

std::shared_ptr<MessagePairInterface> PublisherInterface::get_message_pair() const {
  return this->message_pair_;
}

void PublisherInterface::set_message_pair(const std::shared_ptr<MessagePairInterface>& message_pair) {
  if (message_pair == nullptr) {
    throw exceptions::NullPointerException("Provide a valid pointer");
  }
  this->message_pair_ = message_pair;
}

PublisherType PublisherInterface::get_type() const {
  return this->type_;
}
}// namespace modulo_core::communication
