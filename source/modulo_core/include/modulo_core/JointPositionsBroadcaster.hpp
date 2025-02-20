#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <modulo_interfaces/msg/joint_positions.hpp>
#include <modulo_interfaces/msg/joint_positions_collection.hpp>
#include <state_representation/space/joint/JointPositions.hpp>

namespace modulo_core {

class JointPositionsBroadcaster {
public:
  template<class NodeT, class AllocatorT = std::allocator<void>>
  JointPositionsBroadcaster(
      NodeT&& node, const rclcpp::QoS& qos = rclcpp::QoS(1).transient_local(),
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
          []() {
            rclcpp::PublisherOptionsWithAllocator<AllocatorT> options;
            options.qos_overriding_options = rclcpp::QosOverridingOptions{
                rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
            /*
              This flag disables intra-process communication when the JointPositionsBroadcaster is constructed
              using an existing node handle which happens to be a component (in rclcpp terminology).
              Required until rclcpp intra-process communication supports transient_local QoS durability.
            */
            options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
            return options;
          }())
      : JointPositionsBroadcaster(
            rclcpp::node_interfaces::get_node_parameters_interface(node),
            rclcpp::node_interfaces::get_node_topics_interface(node), qos, options) {}

  template<class AllocatorT = std::allocator<void>>
  JointPositionsBroadcaster(
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
      const rclcpp::QoS& qos = rclcpp::QoS(1).transient_local(),
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options = []() {
        rclcpp::PublisherOptionsWithAllocator<AllocatorT> options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions{
            rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
        /*
          This flag disables intra-process communication when the JointPositionsBroadcaster is constructed
          using an existing node handle which happens to be a component (in rclcpp terminology).
          Required until rclcpp intra-process communication supports transient_local QoS durability.
        */
        options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        return options;
      }()) {
    publisher_ = rclcpp::create_publisher<modulo_interfaces::msg::JointPositionsCollection>(
        node_parameters, node_topics, "/joint_positions", qos, options);
  }

  template<typename T>
  void send(const T& joint_positions);

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
  send({joint_positions});
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
