#include "modulo_core/communication/SubscriptionHandler.hpp"

#include <utility>

namespace modulo_core::communication {

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Bool>)>
SubscriptionHandler<std_msgs::msg::Bool>::get_translated_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Bool> message) {
    try {
      this->get_message_pair()->template read<std_msgs::msg::Bool, bool>(*message);
      this->user_callback_();
    } catch (...) {
      this->handle_callback_exceptions();
    }
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Float64>)>
SubscriptionHandler<std_msgs::msg::Float64>::get_translated_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Float64> message) {
    try {
      this->get_message_pair()->template read<std_msgs::msg::Float64, double>(*message);
      this->user_callback_();
    } catch (...) {
      this->handle_callback_exceptions();
    }
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Float64MultiArray>)>
SubscriptionHandler<std_msgs::msg::Float64MultiArray>::get_translated_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Float64MultiArray> message) {
    try {
      this->get_message_pair()->template read<std_msgs::msg::Float64MultiArray, std::vector<double>>(*message);
      this->user_callback_();
    } catch (...) {
      this->handle_callback_exceptions();
    }
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::Int32>)>
SubscriptionHandler<std_msgs::msg::Int32>::get_translated_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::Int32> message) {
    try {
      this->get_message_pair()->template read<std_msgs::msg::Int32, int>(*message);
      this->user_callback_();
    } catch (...) {
      this->handle_callback_exceptions();
    }
  };
}

template<>
std::function<void(const std::shared_ptr<std_msgs::msg::String>)>
SubscriptionHandler<std_msgs::msg::String>::get_translated_callback() {
  return [this](const std::shared_ptr<std_msgs::msg::String> message) {
    try {
      this->get_message_pair()->template read<std_msgs::msg::String, std::string>(*message);
      this->user_callback_();
    } catch (...) {
      this->handle_callback_exceptions();
    }
  };
}

template<>
std::function<void(const std::shared_ptr<EncodedState>)> SubscriptionHandler<EncodedState>::get_translated_callback() {
  return [this](const std::shared_ptr<EncodedState> message) {
    try {
      this->get_message_pair()->template read<EncodedState, state_representation::State>(*message);
      this->user_callback_();
    } catch (...) {
      this->handle_callback_exceptions();
    }
  };
}

}// namespace modulo_core::communication
