#pragma once

#include <functional>
#include <memory>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <modulo_interfaces/msg/joint_positions.hpp>
#include <modulo_interfaces/msg/joint_positions_collection.hpp>
#include <state_representation/space/joint/JointPositions.hpp>

namespace modulo_core {

namespace detail {
template<class AllocatorT = std::allocator<void>>
rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> get_default_transform_listener_sub_options() {
  rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
  /*
      This flag disables intra-process communication when the JointPositionsListener is constructed
      using an existing node handle which happens to be a component (in rclcpp terminology).
      Required until rclcpp intra-process communication supports transient_local QoS durability.
  */
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  return options;
}
}// namespace detail

/**
 * @class JointPositionsListener
 * @brief The JointPositionsListener is a TF2 style class that listens to the fixed /joint_positions topic and allows to
 * lookup messages from a buffer.
 */
class JointPositionsListener {
public:
  /**
   * @brief Constructor of the JointPositionsListener with a node
   */
  template<class NodeT, class AllocatorT = std::allocator<void>>
  JointPositionsListener(
      NodeT&& node, bool spin_thread = false, const rclcpp::QoS& qos = rclcpp::QoS(100).transient_local(),
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options =
          detail::get_default_transform_listener_sub_options<AllocatorT>())
      : JointPositionsListener(
            node->get_node_base_interface(), node->get_node_parameters_interface(), node->get_node_topics_interface(),
            spin_thread, qos, options) {}

  /**
   * @brief Constructor of the JointPositionsListener with node interfaces
   */
  template<class AllocatorT = std::allocator<void>>
  JointPositionsListener(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics, bool spin_thread = false,
      const rclcpp::QoS& qos = rclcpp::QoS(100).transient_local(),
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options =
          detail::get_default_transform_listener_sub_options<AllocatorT>()) {
    init(node_base, node_parameters, node_topics, spin_thread, qos, options);
  }

  virtual ~JointPositionsListener();

  /**
   * @brief Look up JointPositions object by it's name
   * @tparam T Type of the JointPositions object
   * @param name The name of the JointPositions object to lookup
   */
  template<typename T>
  T lookup(const std::string& name) const;

  /**
   * @brief Get a vector of all buffered JointPositions objects
   * @tparam T Type of the JointPositions objects
   */
  template<typename T>
  std::vector<T> get_buffer() const;

private:
  template<class AllocatorT = std::allocator<void>>
  void init(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics, bool spin_thread, const rclcpp::QoS& qos,
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options) {
    spin_thread_ = spin_thread;

    auto callback = [this](const std::shared_ptr<modulo_interfaces::msg::JointPositionsCollection> message) {
      this->subscription_callback(message);
    };

    if (spin_thread_) {
      // Create new callback group for subscription
      callback_group_ = node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
      // Duplicate to modify option of subscription
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> new_options = options;
      new_options.callback_group = callback_group_;

      subscription_ = rclcpp::create_subscription<modulo_interfaces::msg::JointPositionsCollection>(
          node_parameters, node_topics, "/joint_positions", qos, std::move(callback), new_options);

      // Create executor with dedicated thread to spin.
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_callback_group(callback_group_, node_base);
      dedicated_listener_thread_ = std::make_unique<std::thread>([&]() { executor_->spin(); });
    } else {
      subscription_ = rclcpp::create_subscription<modulo_interfaces::msg::JointPositionsCollection>(
          node_parameters, node_topics, "/joint_positions", qos, std::move(callback), options);
    }
  }

  void subscription_callback(const std::shared_ptr<modulo_interfaces::msg::JointPositionsCollection> message);

  bool spin_thread_;
  std::unique_ptr<std::thread> dedicated_listener_thread_;
  rclcpp::Executor::SharedPtr executor_;

  std::shared_ptr<rclcpp::Node> optional_default_node_;
  std::shared_ptr<rclcpp::Subscription<modulo_interfaces::msg::JointPositionsCollection>> subscription_;
  std::shared_ptr<rclcpp::CallbackGroup> callback_group_;

  std::vector<modulo_interfaces::msg::JointPositions> buffer_;
};
}// namespace modulo_core
