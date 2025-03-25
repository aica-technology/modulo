#pragma once

#include <rclcpp/rclcpp.hpp>

namespace modulo_core {

namespace detail {
template<class OptionsT>
OptionsT get_default_joint_positions_options() {
  OptionsT options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
  /*
      This flag disables intra-process communication.
      Required until rclcpp intra-process communication supports transient_local QoS durability.
  */
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  return options;
}
}// namespace detail
}// namespace modulo_core
