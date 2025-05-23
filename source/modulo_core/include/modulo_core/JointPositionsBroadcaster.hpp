#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <modulo_interfaces/msg/joint_positions.hpp>
#include <modulo_interfaces/msg/joint_positions_collection.hpp>
#include <state_representation/space/joint/JointPositions.hpp>

#include "modulo_core/joint_positions_options.hpp"

namespace modulo_core {

/**
 * @class JointPositionsBroadcaster
 * @brief The JointPositionsBroadcaster is a TF2 style class that publishes a collection of JointPositions messages to
 * the fixed /joint_positions topic.
 */
class JointPositionsBroadcaster {
public:
  /**
   * @brief Constructor of the JointPositionsBroadcaster with a node
   */
  template<class NodeT, class AllocatorT = std::allocator<void>>
  JointPositionsBroadcaster(
      NodeT&& node, const rclcpp::QoS& qos = rclcpp::QoS(1).transient_local(),
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
          detail::get_default_joint_positions_options<rclcpp::PublisherOptionsWithAllocator<AllocatorT>>())
      : JointPositionsBroadcaster(
            rclcpp::node_interfaces::get_node_parameters_interface(node),
            rclcpp::node_interfaces::get_node_topics_interface(node), qos, options) {}

  /**
   * @brief Constructor of the JointPositionsBroadcaster with node interfaces
   */
  template<class AllocatorT = std::allocator<void>>
  JointPositionsBroadcaster(
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
      const rclcpp::QoS& qos = rclcpp::QoS(1).transient_local(),
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
          detail::get_default_joint_positions_options<rclcpp::PublisherOptionsWithAllocator<AllocatorT>>()) {
    publisher_ = rclcpp::create_publisher<modulo_interfaces::msg::JointPositionsCollection>(
        node_parameters, node_topics, "/joint_positions", qos, options);
  }

  /**
   * @brief Send a JointPositions object
   * @tparam T Type of the JointPositions object
   * @param joint_positions The JointPositions object to send
   */
  template<typename T>
  void send(const T& joint_positions);

  /**
   * @brief Send a vector of JointPositions objects
   * @tparam T Type of the JointPositions objects
   * @param joint_positions The vector of JointPositions objects to send
   */
  template<typename T>
  void send(const std::vector<T>& joint_positions);

private:
  template<typename T>
  void send(
      const std::vector<T>& joint_positions, std::function<std::string(T)> get_name,
      std::function<modulo_interfaces::msg::JointPositions(T)> translator);

  std::shared_ptr<rclcpp::Publisher<modulo_interfaces::msg::JointPositionsCollection>> publisher_;
  modulo_interfaces::msg::JointPositionsCollection net_message_;
};

template<typename T>
inline void JointPositionsBroadcaster::send(const T& joint_positions) {
  send(std::vector<T>{joint_positions});
}

template<typename T>
inline void JointPositionsBroadcaster::send(
    const std::vector<T>& joint_positions, std::function<std::string(T)> get_name,
    std::function<modulo_interfaces::msg::JointPositions(T)> translator) {
  for (auto it_in = joint_positions.begin(); it_in != joint_positions.end(); ++it_in) {
    bool match_found = false;
    for (auto it_msg = net_message_.joint_positions.begin(); it_msg != net_message_.joint_positions.end(); ++it_msg) {
      if (get_name(*it_in) == it_msg->header.frame_id) {
        *it_msg = translator(*it_in);
        match_found = true;
        break;
      }
    }
    if (!match_found) {
      net_message_.joint_positions.push_back(translator(*it_in));
    }
  }
  publisher_->publish(net_message_);
}
}// namespace modulo_core
