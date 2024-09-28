#pragma once

#include <rclcpp/logging.hpp>

#include "modulo_core/communication/PublisherInterface.hpp"

#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

namespace modulo_core::communication {

/**
 * @class PublisherHandler
 * @brief The PublisherHandler handles different types of ROS publishers to activate, deactivate and publish data with
 * those publishers.
 * @tparam PubT The ROS publisher type
 * @tparam MsgT The ROS message type of the ROS publisher
 */
template<typename PubT, typename MsgT>
class PublisherHandler : public PublisherInterface {
public:
  /**
   * @brief Constructor with the publisher type and the pointer to the ROS publisher.
   * @param type The publisher type
   * @param publisher The pointer to the ROS publisher
   */
  PublisherHandler(PublisherType type, std::shared_ptr<PubT> publisher);

  /**
   * @brief Destructor to explicitly reset the publisher pointer.
   */
  ~PublisherHandler() override;

  /**
     * @brief Activate ROS publisher.
     * @throws modulo_core::exceptions::NullPointerException if the publisher is null
     */
  virtual void activate() override;

  /**
     * @brief Deactivate ROS publisher.
     * @throws modulo_core::exceptions::NullPointerException if the publisher is null
     */
  virtual void deactivate() override;

  /**
   * @brief Publish the ROS message through the ROS publisher.
   * @param message The ROS message to publish
   * @throws modulo_core::exceptions::NullPointerException if the publisher pointer is null
   */
  void publish() override;

  /**
   * @brief Publish the ROS message through the ROS publisher.
   * @param message The ROS message to publish
   * @throws modulo_core::exceptions::NullPointerException if the publisher pointer is null
   */
  void publish(const MsgT& message) const;

  /**
   * @brief Create a PublisherInterface instance from the current PublisherHandler.
   * @param message_pair The message pair of the PublisherInterface
   */
  std::shared_ptr<PublisherInterface>
  create_publisher_interface(const std::shared_ptr<MessagePairInterface>& message_pair);

private:
  std::shared_ptr<PubT> publisher_;///< The ROS publisher
};

template<typename PubT, typename MsgT>
PublisherHandler<PubT, MsgT>::PublisherHandler(PublisherType type, std::shared_ptr<PubT> publisher)
    : PublisherInterface(type), publisher_(std::move(publisher)) {}

template<typename PubT, typename MsgT>
PublisherHandler<PubT, MsgT>::~PublisherHandler() {
  this->publisher_.reset();
}

template<typename PubT, typename MsgT>
void PublisherHandler<PubT, MsgT>::activate() {
  if constexpr (std::same_as<PubT, rclcpp_lifecycle::LifecyclePublisher<MsgT>>) {
    if (this->publisher_ == nullptr) {
      throw exceptions::NullPointerException("Publisher not set");
    }
    try {
      this->publisher_->on_activate();
    } catch (const std::exception& ex) {
      throw exceptions::CoreException(ex.what());
    }
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("PublisherHandler"), "Only LifecyclePublishers can be deactivated");
  }
}

template<typename PubT, typename MsgT>
void PublisherHandler<PubT, MsgT>::deactivate() {
  if constexpr (std::same_as<PubT, rclcpp_lifecycle::LifecyclePublisher<MsgT>>) {
    if (this->publisher_ == nullptr) {
      throw exceptions::NullPointerException("Publisher not set");
    }
    try {
      this->publisher_->on_deactivate();
    } catch (const std::exception& ex) {
      throw exceptions::CoreException(ex.what());
    }
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("PublisherHandler"), "Only LifecyclePublishers can be deactivated");
  }
}

template<typename PubT, typename MsgT>
void PublisherHandler<PubT, MsgT>::publish() {
  if (this->message_pair_ == nullptr) {
    throw exceptions::NullPointerException("Message pair is not set, cannot deduce message type");
  }
  if (this->message_pair_->get_type() == MessageType::CUSTOM_MESSAGE) {
    // explicitly check for EncodedState at complile time to hide the otherwise incompatible code block for EncodedState
    if constexpr (!std::same_as<MsgT, EncodedState> && CustomDataT<MsgT>) {
      publish(this->message_pair_->write<MsgT, MsgT>());
    }
  } else {
    PublisherInterface::publish();
  }
}

template<typename PubT, typename MsgT>
void PublisherHandler<PubT, MsgT>::publish(const MsgT& message) const {
  if (this->publisher_ == nullptr) {
    throw exceptions::NullPointerException("Publisher not set");
  }
  try {
    this->publisher_->publish(message);
  } catch (const std::exception& ex) {
    throw exceptions::CoreException(ex.what());
  }
}

template<typename PubT, typename MsgT>
std::shared_ptr<PublisherInterface>
PublisherHandler<PubT, MsgT>::create_publisher_interface(const std::shared_ptr<MessagePairInterface>& message_pair) {
  std::shared_ptr<PublisherInterface> publisher_interface;
  try {
    publisher_interface = std::shared_ptr<PublisherInterface>(this->shared_from_this());
  } catch (const std::exception& ex) {
    throw exceptions::CoreException(ex.what());
  }
  publisher_interface->set_message_pair(message_pair);
  return publisher_interface;
}
}// namespace modulo_core::communication
