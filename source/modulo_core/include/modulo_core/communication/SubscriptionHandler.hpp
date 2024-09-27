#pragma once

#include "modulo_core/communication/SubscriptionInterface.hpp"

namespace modulo_core::communication {

/**
 * @class SubscriptionHandler
 * @brief The SubscriptionHandler handles different types of ROS subscriptions to receive data from those subscriptions.
 * @tparam MsgT The ROS message type of the ROS subscription
 */
template<typename MsgT>
class SubscriptionHandler : public SubscriptionInterface {
public:
  /**
   * @brief Constructor with the message pair.
   * @param message_pair The pointer to the message pair with the data that should be updated through the subscription
   */
  explicit SubscriptionHandler(std::shared_ptr<MessagePairInterface> message_pair = nullptr);

  /**
   * @brief Destructor to explicitly reset the subscription pointer.
   */
  ~SubscriptionHandler() override;

  /**
   * @brief Getter of the ROS subscription.
   */
  [[nodiscard]] std::shared_ptr<rclcpp::Subscription<MsgT>> get_subscription() const;

  /**
   * @brief Setter of the ROS subscription.
   * @param subscription The ROS subscription
   * @throws modulo_core::exceptions::NullPointerException if the provided subscription pointer is null
   */
  void set_subscription(const std::shared_ptr<rclcpp::Subscription<MsgT>>& subscription);

  /**
   * @brief Setter of a user callback function to be executed after the subscription callback
   * @param user_callback The ser callback function
   */
  void set_user_callback(const std::function<void()>& user_callback);

  /**
   * @brief Get a callback function that will be associated with the ROS subscription to receive and translate messages.
   */
  std::function<void(const std::shared_ptr<MsgT>)> get_callback();

  /**
   * @brief Get a callback function that will be associated with the ROS subscription to receive and translate messages.
   * @details This variant also takes a user callback function to execute after the message is received and translated.
   * @param user_callback Void callback function for additional logic after the message is received and translated.
   */
  std::function<void(const std::shared_ptr<MsgT>)> get_callback(const std::function<void()>& user_callback);

  /**
   * @brief Create a SubscriptionInterface pointer through an instance of a SubscriptionHandler by providing a ROS
   * subscription.
   * @details This throws a NullPointerException if the ROS subscription is null.
   * @see SubscriptionHandler::set_subscription
   * @param subscription The ROS subscription
   * @return The resulting SubscriptionInterface pointer
   */
  std::shared_ptr<SubscriptionInterface>
  create_subscription_interface(const std::shared_ptr<rclcpp::Subscription<MsgT>>& subscription);

private:
  /**
   * @brief Handle exceptions caught during callback evaluation.
   */
  void handle_callback_exceptions();

  std::shared_ptr<rclcpp::Subscription<MsgT>> subscription_;///< The pointer referring to the ROS subscription
  std::shared_ptr<rclcpp::Clock> clock_;                    ///< ROS clock for throttling log
  std::function<void()> user_callback_ = [] {
  };///< User callback to be executed after the subscription callback
};

template<typename MsgT>
SubscriptionHandler<MsgT>::~SubscriptionHandler() {
  this->subscription_.reset();
}

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
void SubscriptionHandler<MsgT>::set_user_callback(const std::function<void()>& user_callback) {
  this->user_callback_ = user_callback;
}

template<typename MsgT>
std::function<void(const std::shared_ptr<MsgT>)>
SubscriptionHandler<MsgT>::get_callback(const std::function<void()>& user_callback) {
  this->set_user_callback(user_callback);
  return this->get_callback();
}

template<typename MsgT>
std::shared_ptr<SubscriptionInterface> SubscriptionHandler<MsgT>::create_subscription_interface(
    const std::shared_ptr<rclcpp::Subscription<MsgT>>& subscription) {
  this->set_subscription(subscription);
  return std::shared_ptr<SubscriptionInterface>(this->shared_from_this());
}

template<typename MsgT>
void SubscriptionHandler<MsgT>::handle_callback_exceptions() {
  try {
    // re-throw the original exception
    throw;
  } catch (const exceptions::CoreException& ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
        rclcpp::get_logger("SubscriptionHandler"), *this->clock_, 1000,
        "Exception in subscription callback: " << ex.what());
  } catch (const std::exception& ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
        rclcpp::get_logger("SubscriptionHandler"), *this->clock_, 1000,
        "Unhandled exception in subscription user callback: " << ex.what());
  }
}

}// namespace modulo_core::communication
