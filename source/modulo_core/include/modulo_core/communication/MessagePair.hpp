#pragma once

#include <rclcpp/clock.hpp>

#include "modulo_core/communication/MessagePairInterface.hpp"
#include "modulo_core/exceptions/NullPointerException.hpp"
#include "modulo_core/translators/message_readers.hpp"
#include "modulo_core/translators/message_writers.hpp"

namespace modulo_core::communication {

/**
 * @class MessagePair
 * @brief The MessagePair stores a pointer to a variable and translates the value of this pointer back and forth between
 * the corresponding ROS messages.
 * @tparam MsgT ROS message type of the MessagePair
 * @tparam DataT Data type corresponding to the ROS message type
 */
template<typename MsgT, typename DataT>
class MessagePair : public MessagePairInterface {
public:
  /**
   * @brief Constructor of the MessagePair.
   * @param data The pointer referring to the data stored in the MessagePair
   * @param clock The ROS clock for translating messages
   */
  MessagePair(std::shared_ptr<DataT> data, std::shared_ptr<rclcpp::Clock> clock);

  /**
   * @brief Write the value of the data pointer to a ROS message.
   * @return The value of the data pointer as a ROS message
   * @throws modulo_core::exceptions::NullPointerException if the data pointer is null
   * @throws modulo_core::exceptions::MessageTranslationException if the data could not be written to message
   */
  [[nodiscard]] MsgT write_message() const;

  /**
   * @brief Read a ROS message and store the value in the data pointer.
   * @param message The ROS message to read
   * @throws modulo_core::exceptions::NullPointerException if the data pointer is null
   * @throws modulo_core::exceptions::MessageTranslationException if the message could not be read
   */
  void read_message(const MsgT& message);

  /**
   * @brief Get the data pointer.
   */
  [[nodiscard]] std::shared_ptr<DataT> get_data() const;

  /**
   * @brief Set the data pointer.
   * @throws modulo_core::exceptions::NullPointerException if the provided data pointer is null
   */
  void set_data(const std::shared_ptr<DataT>& data);

private:
  std::shared_ptr<DataT> data_; ///< Pointer referring to the data stored in the MessagePair
  std::shared_ptr<rclcpp::Clock> clock_; ///< ROS clock for translating messages
};

template<typename MsgT, typename DataT>
inline MsgT MessagePair<MsgT, DataT>::write_message() const {
  if (this->data_ == nullptr) {
    throw exceptions::NullPointerException("The message pair data is not set, nothing to write");
  }
  auto message = MsgT();
  translators::write_message(message, *this->data_, clock_->now());
  return message;
}

template<>
inline EncodedState MessagePair<EncodedState, state_representation::State>::write_message() const {
  if (this->data_ == nullptr) {
    throw exceptions::NullPointerException("The message pair data is not set, nothing to write");
  }
  auto message = EncodedState();
  translators::write_message(message, this->data_, clock_->now());
  return message;
}

template<typename MsgT, typename DataT>
inline void MessagePair<MsgT, DataT>::read_message(const MsgT& message) {
  if (this->data_ == nullptr) {
    throw exceptions::NullPointerException("The message pair data is not set, nothing to read");
  }
  translators::read_message(*this->data_, message);
}

template<>
inline void MessagePair<EncodedState, state_representation::State>::read_message(const EncodedState& message) {
  if (this->data_ == nullptr) {
    throw exceptions::NullPointerException("The message pair data is not set, nothing to read");
  }
  translators::read_message(this->data_, message);
}

template<typename MsgT, typename DataT>
inline std::shared_ptr<DataT> MessagePair<MsgT, DataT>::get_data() const {
  return this->data_;
}

template<typename MsgT, typename DataT>
inline void MessagePair<MsgT, DataT>::set_data(const std::shared_ptr<DataT>& data) {
  if (data == nullptr) {
    throw exceptions::NullPointerException("Provide a valid pointer");
  }
  this->data_ = data;
}

template<typename DataT>
inline std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<DataT>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<EncodedState, state_representation::State>>(
      std::dynamic_pointer_cast<state_representation::State>(data), clock);
}

template<>
inline std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<bool>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::Bool, bool>>(data, clock);
}

template<>
inline std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<double>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::Float64, double>>(data, clock);
}

template<>
inline std::shared_ptr<MessagePairInterface> make_shared_message_pair(
    const std::shared_ptr<std::vector<double>>& data, const std::shared_ptr<rclcpp::Clock>& clock
) {
  return std::make_shared < MessagePair<std_msgs::msg::Float64MultiArray, std::vector < double>>>(data, clock);
}

template<>
inline std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<int>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::Int32, int>>(data, clock);
}

template<>
inline std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<std::string>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::String, std::string>>(data, clock);
}
}// namespace modulo_core::communication
