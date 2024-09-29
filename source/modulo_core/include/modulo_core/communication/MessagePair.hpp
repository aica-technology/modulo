#pragma once

#include <rclcpp/clock.hpp>

#include "modulo_core/communication/MessagePairInterface.hpp"
#include "modulo_core/concepts.hpp"
#include "modulo_core/translators/message_readers.hpp"
#include "modulo_core/translators/message_writers.hpp"

namespace modulo_core::communication {

using namespace modulo_core::concepts;

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
   * @brief Constructor of the MessagePair that requires custom message types only
   * @param data The pointer referring to the data stored in the MessagePair
   * @param clock The ROS clock for translating messages
   */
  MessagePair(std::shared_ptr<DataT> data, std::shared_ptr<rclcpp::Clock> clock)
    requires CustomDataT<MsgT> && CustomDataT<DataT>
      : MessagePairInterface(MessageType::CUSTOM_MESSAGE), data_(std::move(data)), clock_(std::move(clock)) {}

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

protected:
  [[nodiscard]] MsgT write_translated_message() const;
  [[nodiscard]] MsgT write_encoded_message() const;
  [[nodiscard]] MsgT write_raw_message() const;

  void read_translated_message(const MsgT& message);
  void read_encoded_message(const MsgT& message);
  void read_raw_message(const MsgT& message);

private:
  std::shared_ptr<DataT> data_;         ///< Pointer referring to the data stored in the MessagePair
  std::shared_ptr<rclcpp::Clock> clock_;///< ROS clock for translating messages
};

template<typename MsgT, typename DataT>
inline MsgT MessagePair<MsgT, DataT>::write_message() const {
  if (this->data_ == nullptr) {
    throw exceptions::NullPointerException("The message pair data is not set, nothing to write");
  }

  MsgT message;
  if constexpr (TranslatedDataT<MsgT> && !CustomDataT<DataT>) {
    message = write_translated_message();
  } else if constexpr (std::same_as<MsgT, EncodedState>) {
    message = write_encoded_message();
  } else if constexpr (CustomDataT<DataT>) {
    message = write_raw_message();
  } else {
    static_assert(false, "The message types for the message pair are not supported.");
  }
  return message;
}

template<typename MsgT, typename DataT>
inline MsgT MessagePair<MsgT, DataT>::write_translated_message() const {
  auto message = MsgT();
  translators::write_message(message, *this->data_, clock_->now());
  return message;
}

template<>
inline EncodedState MessagePair<EncodedState, state_representation::State>::write_encoded_message() const {
  auto message = EncodedState();
  translators::write_message(message, this->data_, clock_->now());
  return message;
}

template<typename MsgT, typename DataT>
inline MsgT MessagePair<MsgT, DataT>::write_raw_message() const {
  return *this->data_;
}

template<typename MsgT, typename DataT>
inline void MessagePair<MsgT, DataT>::read_message(const MsgT& message) {
  if (this->data_ == nullptr) {
    throw exceptions::NullPointerException("The message pair data is not set, nothing to read");
  }

  if constexpr (std::same_as<MsgT, EncodedState>) {
    read_encoded_message(message);
  } else if constexpr (std::same_as<MsgT, DataT> || (CustomDataT<MsgT> && CustomDataT<DataT>) ) {
    read_raw_message(message);
  } else if constexpr (TranslatedDataT<MsgT> || CoreDataT<MsgT>) {
    read_translated_message(message);
  } else {
    static_assert(false, "The message types for the message pair are not supported.");
  }
}

template<typename MsgT, typename DataT>
inline void MessagePair<MsgT, DataT>::read_translated_message(const MsgT& message) {
  translators::read_message(*this->data_, message);
}

template<>
inline void MessagePair<EncodedState, state_representation::State>::read_encoded_message(const EncodedState& message) {
  translators::read_message(this->data_, message);
}

template<typename MsgT, typename DataT>
inline void MessagePair<MsgT, DataT>::read_raw_message(const MsgT& message) {
  *this->data_ = message;
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

template<CoreDataT DataT>
inline std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<DataT>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<EncodedState, state_representation::State>>(
      std::dynamic_pointer_cast<state_representation::State>(data), clock);
}

template<CustomDataT DataT>
inline std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<DataT>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<DataT, DataT>>(data, clock);
}

template<typename DataT = bool>
inline std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<bool>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::Bool, bool>>(data, clock);
}

template<typename DataT = double>
inline std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<double>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::Float64, double>>(data, clock);
}

template<typename DataT = std::vector<double>>
inline std::shared_ptr<MessagePairInterface> make_shared_message_pair(
    const std::shared_ptr<std::vector<double>>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::Float64MultiArray, std::vector<double>>>(data, clock);
}

template<typename DataT = int>
inline std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<int>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::Int32, int>>(data, clock);
}

template<typename DataT = std::string>
inline std::shared_ptr<MessagePairInterface>
make_shared_message_pair(const std::shared_ptr<std::string>& data, const std::shared_ptr<rclcpp::Clock>& clock) {
  return std::make_shared<MessagePair<std_msgs::msg::String, std::string>>(data, clock);
}
}// namespace modulo_core::communication
