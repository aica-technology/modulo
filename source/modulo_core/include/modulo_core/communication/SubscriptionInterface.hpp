#pragma once

#include <rclcpp/subscription.hpp>

#include "modulo_core/communication/MessagePair.hpp"
#include "modulo_core/exceptions.hpp"

namespace modulo_core::communication {

// forward declaration of derived SubscriptionHandler class
template<typename MsgT>
class SubscriptionHandler;

/**
 * @class SubscriptionInterface
 * @brief Interface class to enable non-templated subscriptions with ROS subscriptions from derived SubscriptionHandler
 * instances through dynamic down-casting.
 */
class SubscriptionInterface : public std::enable_shared_from_this<SubscriptionInterface> {
public:
  /**
   * @brief Constructor with the message pair.
   * @param message_pair The pointer to the message pair with the data that should be updated through the subscription
   */
  explicit SubscriptionInterface(std::shared_ptr<MessagePairInterface> message_pair = nullptr);

  /**
   * @brief Copy constructor from another SubscriptionInterface.
   */
  SubscriptionInterface(const SubscriptionInterface& subscription) = default;

  /**
   * @brief Default virtual destructor.
   */
  virtual ~SubscriptionInterface() = default;

  /**
   * @brief Get a pointer to a derived SubscriptionHandler instance from a SubscriptionInterface pointer.
   * @details If a SubscriptionInterface pointer is used to address a derived SubscriptionHandler instance, this method
   * will return a pointer to that derived instance through dynamic down-casting. The downcast will fail if the base
   * SubscriptionInterface object has no reference count (if the object is not owned by any pointer), or if the derived
   * object is not a correctly typed instance of a SubscriptionHandler. By default, an InvalidPointerCastException is
   * thrown when the downcast fails. If this validation is disabled by setting the validate_pointer flag to false, it
   * will not throw an exception and instead return a null pointer.
   * @tparam PubT The ROS publisher type
   * @tparam MsgT The ROS message type
   * @throws modulo_core::exceptions::InvalidPointerException if the base SubscriptionInterface object has no reference
   * count and validate_pointer is set to true
   * @throws modulo_core::exceptions::InvalidPointerCastException if the derived object from the dynamic down-casting
   * is not a correctly typed instance of a SubscriptionHandler
   * @param validate_pointer If true, throw an exception when down-casting fails
   * @return A pointer to a derived SubscriptionHandler instance of the desired type, or a null pointer
   * if down-casting failed and validate_pointer was set to false.
   */
  template<typename MsgT>
  std::shared_ptr<SubscriptionHandler<MsgT>> get_handler(bool validate_pointer = true);

  /**
   * @brief Get the pointer to the message pair of the SubscriptionInterface.
   */
  [[nodiscard]] std::shared_ptr<MessagePairInterface> get_message_pair() const;

  /**
   * @brief Set the pointer to the message pair of the SubscriptionInterface.
   * @throws modulo_core::exceptions::NullPointerException if the provided message pair pointer is null
   */
  void set_message_pair(const std::shared_ptr<MessagePairInterface>& message_pair);

protected:
  std::shared_ptr<MessagePairInterface> message_pair_;///< The pointer to the stored MessagePair instance
};

template<typename MsgT>
inline std::shared_ptr<SubscriptionHandler<MsgT>> SubscriptionInterface::get_handler(bool validate_pointer) {
  std::shared_ptr<SubscriptionHandler<MsgT>> subscription_ptr;
  try {
    subscription_ptr = std::dynamic_pointer_cast<SubscriptionHandler<MsgT>>(this->shared_from_this());
  } catch (const std::exception& ex) {
    if (validate_pointer) {
      throw exceptions::InvalidPointerException("Subscription interface is not managed by a valid pointer");
    }
  }
  if (subscription_ptr == nullptr && validate_pointer) {
    throw exceptions::InvalidPointerCastException(
        "Unable to cast subscription interface to a subscription pointer of requested type");
  }
  return subscription_ptr;
}
}// namespace modulo_core::communication
