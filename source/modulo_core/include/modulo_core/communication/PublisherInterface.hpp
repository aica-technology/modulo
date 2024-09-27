#pragma once

#include <memory>

#include "modulo_core/communication/MessagePair.hpp"
#include "modulo_core/communication/PublisherType.hpp"
#include "modulo_core/exceptions.hpp"

namespace modulo_core::communication {

// forward declaration of derived Publisher class
template<typename PubT, typename MsgT>
class PublisherHandler;

/**
 * @class PublisherInterface
 * @brief Interface class to enable non-templated activating/deactivating/publishing of ROS publishers from derived
 * PublisherHandler instances through dynamic down-casting.
 */
class PublisherInterface : public std::enable_shared_from_this<PublisherInterface> {
public:
  /**
   * @brief Constructor with the message type and message pair.
   * @param type The type of the publisher interface
   * @param message_pair The message pair with the data to be published
   */
  explicit PublisherInterface(PublisherType type, std::shared_ptr<MessagePairInterface> message_pair = nullptr);

  /**
   * @brief Copy constructor from another PublisherInterface.
   */
  PublisherInterface(const PublisherInterface& publisher) = default;

  /**
   * @brief Default virtual destructor.
   */
  virtual ~PublisherInterface() = default;

  /**
   * @brief Get a pointer to a derived PublisherHandler instance from a PublisherInterface pointer.
   * @details If a PublisherInterface pointer is used to address a derived PublisherHandler instance, this method will
   * return a pointer to that derived instance through dynamic down-casting. The downcast will fail if the base
   * PublisherInterface object has no reference count (if the object is not owned by any pointer), or if the derived
   * object is not a correctly typed instance of a PublisherHandler. By default, an InvalidPointerCastException is
   * thrown when the downcast fails. If this validation is disabled by setting the validate_pointer flag to false, it
   * will not throw an exception and instead return a null pointer.
   * @tparam PubT The ROS publisher type
   * @tparam MsgT The ROS message type
   * @throws modulo_core::exceptions::InvalidPointerException if the base PublisherInterface object has no reference
   * count and validate_pointer is set to true
   * @throws modulo_core::exceptions::InvalidPointerCastException if the derived object from the dynamic down-casting
   * is not a correctly typed instance of a PublisherHandler
   * @param validate_pointer If true, throw an exception when down-casting fails
   * @return A pointer to a derived PublisherHandler instance of the desired type, or a null pointer
   * if down-casting failed and validate_pointer was set to false.
   */
  template<typename PubT, typename MsgT>
  std::shared_ptr<PublisherHandler<PubT, MsgT>> get_handler(bool validate_pointer = true);

  /**
   * @brief Activate ROS publisher of a derived PublisherHandler instance through the PublisherInterface pointer.
   * @details This throws an InvalidPointerCastException if the PublisherInterface does not point to a valid
   * PublisherHandler instance or if the type of the message pair does not match the type of the PublisherHandler
   * instance.
   * @see PublisherInterface::get_handler
   * @throws modulo_core::exceptions::NullPointerException if the message pair pointer is null
   */
  void activate();

  /**
   * @brief Deactivate ROS publisher of a derived PublisherHandler instance through the PublisherInterface pointer.
   * @details This throws an InvalidPointerCastException if the PublisherInterface does not point to a valid
   * PublisherHandler instance or if the type of the message pair does not match the type of the PublisherHandler
   * instance.
   * @see PublisherInterface::get_handler
   * @throws modulo_core::exceptions::NullPointerException if the message pair pointer is null
   */
  void deactivate();

  /**
   * @brief Publish the data stored in the message pair through the ROS publisher of a derived PublisherHandler
   * instance through the PublisherInterface pointer.
   * @details This throws an InvalidPointerCastException if the PublisherInterface does not point to a valid
   * PublisherHandler instance or if the type of the message pair does not match the type of the PublisherHandler
   * instance.
   * @see PublisherInterface::get_handler
   * @throws modulo_core::exceptions::CoreException if the publishing failed for some reason
   * (translation, null pointer, pointer cast, ...)
   */
  void publish();

  /**
   * @brief Get the pointer to the message pair of the PublisherInterface.
   */
  [[nodiscard]] std::shared_ptr<MessagePairInterface> get_message_pair() const;

  /**
   * @brief Set the pointer to the message pair of the PublisherInterface.
   * @throws modulo_core::exceptions::NullPointerException if the provided message pair pointer is null
   */
  void set_message_pair(const std::shared_ptr<MessagePairInterface>& message_pair);

  /**
   * @brief Get the type of the publisher interface.
   * @see PublisherType
   */
  PublisherType get_type() const;

private:
  /**
   * @brief Publish the data stored in the message pair through the ROS publisher of a derived PublisherHandler instance
   * through the PublisherInterface pointer.
   * @details This throws an InvalidPointerCastException if the PublisherInterface does not point to a valid
   * PublisherHandler instance or if the type of the message pair does not match the type of the PublisherHandler
   * instance.
   * @see PublisherInterface::get_handler
   * @tparam MsgT Message type of the PublisherHandler
   * @param message The ROS message to publish through the ROS publisher
   */
  template<typename MsgT>
  void publish(const MsgT& message);

  PublisherType type_;                                ///< The type of the publisher interface
  std::shared_ptr<MessagePairInterface> message_pair_;///< The pointer to the stored MessagePair instance
};

template<typename PubT, typename MsgT>
inline std::shared_ptr<PublisherHandler<PubT, MsgT>> PublisherInterface::get_handler(bool validate_pointer) {
  std::shared_ptr<PublisherHandler<PubT, MsgT>> publisher_ptr;
  try {
    publisher_ptr = std::dynamic_pointer_cast<PublisherHandler<PubT, MsgT>>(this->shared_from_this());
  } catch (const std::exception& ex) {
    if (validate_pointer) {
      throw exceptions::InvalidPointerException("Publisher interface is not managed by a valid pointer");
    }
  }
  if (publisher_ptr == nullptr && validate_pointer) {
    throw exceptions::InvalidPointerCastException(
        "Unable to cast publisher interface to a publisher pointer of requested type");
  }
  return publisher_ptr;
}
}// namespace modulo_core::communication
