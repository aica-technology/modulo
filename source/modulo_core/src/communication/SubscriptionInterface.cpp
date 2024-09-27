#include "modulo_core/communication/SubscriptionInterface.hpp"

namespace modulo_core::communication {

SubscriptionInterface::SubscriptionInterface(std::shared_ptr<MessagePairInterface> message_pair)
    : message_pair_(std::move(message_pair)) {}

std::shared_ptr<MessagePairInterface> SubscriptionInterface::get_message_pair() const {
  return this->message_pair_;
}

void SubscriptionInterface::set_message_pair(const std::shared_ptr<MessagePairInterface>& message_pair) {
  if (message_pair == nullptr) {
    throw exceptions::NullPointerException("Provide a valid pointer");
  }
  this->message_pair_ = message_pair;
}
}// namespace modulo_core::communication
