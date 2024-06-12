#pragma once

#include <memory>

#include "modulo_core/communication/MessageType.hpp"
#include "modulo_core/exceptions.hpp"

namespace modulo_core::communication {

// forward declaration of derived MessagePair class
template<typename MsgT, typename DataT>
class MessagePair;

/**
 * @class MessagePairInterface
 * @brief Interface class to enable non-templated writing and reading ROS messages from derived MessagePair instances
 * through dynamic down-casting.
 */
class MessagePairInterface : public std::enable_shared_from_this<MessagePairInterface> {
public:
  /**
   * @brief Constructor with the message type
   * @param type The message type of the message pair
   */
  explicit MessagePairInterface(MessageType type);

  /**
   * @brief Default virtual destructor.
   */
  virtual ~MessagePairInterface() = default;

  /**
   * @brief Copy constructor from another MessagePairInterface.
   */
  MessagePairInterface(const MessagePairInterface& message_pair) = default;

  /**
   * @brief Get a pointer to a derived MessagePair instance from a MessagePairInterface pointer.
   * @details If a MessagePairInterface pointer is used to address a derived MessagePair instance, this method will
   * return a pointer to that derived instance through dynamic down-casting. The downcast will fail if the base
   * MessagePairInterface object has no reference count (if the object is not owned by any pointer), or if the derived
   * object is not a correctly typed instance of a MessagePair. By default, an InvalidPointerCastException is thrown
   * when the downcast fails. If this validation is disabled by setting the validate_pointer flag to false, it will not
   * throw an exception and instead return a null pointer.
   * @tparam MsgT The ROS message type of the MessagePair
   * @tparam DataT The data type of the MessagePair
   * @param validate_pointer If true, throw an exception when down-casting fails
   * @throws modulo_core::exceptions::InvalidPointerException if the base MessagePairInterface object has no reference
   * count and validate_pointer is set to true
   * @throws modulo_core::exceptions::InvalidPointerCastException if the derived object from the dynamic down-casting
   * is not a correctly typed instance of a MessagePair
   * @return A pointer to a derived MessagePair instance of the desired type, or a null pointer
   * if down-casting failed and validate_pointer was set to false.
   */
  template<typename MsgT, typename DataT>
  [[nodiscard]] std::shared_ptr<MessagePair<MsgT, DataT>> get_message_pair(bool validate_pointer = true);

  /**
   * @brief Get the ROS message of a derived MessagePair instance through the MessagePairInterface pointer.
   * @details This throws an InvalidPointerCastException if the MessagePairInterface does not point to a valid
   * MessagePair instance or if the specified types does not match the type of the MessagePair instance.
   * @see MessagePairInterface::get_message_pair()
   * @tparam MsgT The ROS message type of the MessagePair
   * @tparam DataT The data type of the MessagePair
   * @return The ROS message containing the data from the underlying MessagePair instance
   */
  template<typename MsgT, typename DataT>
  [[nodiscard]] MsgT write();

  /**
   * @brief Read a ROS message and set the data of the derived MessagePair instance through the MessagePairInterface
   * pointer.
   * @details This throws an InvalidPointerCastException if the MessagePairInterface does not point to a valid
   * MessagePair instance or if the specified types does not match the type of the MessagePair instance.
   * @tparam MsgT The ROS message type of the MessagePair
   * @tparam DataT The data type of the MessagePair
   * @param message The ROS message to read from
   */
  template<typename MsgT, typename DataT>
  void read(const MsgT& message);

  /**
   * @brief Get the MessageType of the MessagePairInterface.
   * @see MessageType
   */
  MessageType get_type() const;

private:
  MessageType type_; ///< The message type of the MessagePairInterface
};

template<typename MsgT, typename DataT>
inline std::shared_ptr<MessagePair<MsgT, DataT>> MessagePairInterface::get_message_pair(bool validate_pointer) {
  std::shared_ptr<MessagePair<MsgT, DataT>> message_pair_ptr;
  try {
    message_pair_ptr = std::dynamic_pointer_cast<MessagePair<MsgT, DataT>>(this->shared_from_this());
  } catch (const std::exception& ex) {
    if (validate_pointer) {
      throw exceptions::InvalidPointerException("Message pair interface is not managed by a valid pointer");
    }
  }
  if (message_pair_ptr == nullptr && validate_pointer) {
    throw exceptions::InvalidPointerCastException(
        "Unable to cast message pair interface to a message pair pointer of requested type");
  }
  return message_pair_ptr;
}

template<typename MsgT, typename DataT>
inline MsgT MessagePairInterface::write() {
  return this->template get_message_pair<MsgT, DataT>()->write_message();
}

template<typename MsgT, typename DataT>
inline void MessagePairInterface::read(const MsgT& message) {
  this->template get_message_pair<MsgT, DataT>()->read_message(message);
}
}// namespace modulo_core::communication
