#pragma once

#include "modulo_components/ComponentInterface.hpp"
#include "modulo_components/Component.hpp"
#include "modulo_components/LifecycleComponent.hpp"

using namespace state_representation;

namespace modulo_components {

class ComponentInterfacePublicInterface : public ComponentInterface {
public:
  template<typename NodeT>
  explicit ComponentInterfacePublicInterface(const std::shared_ptr<NodeT>& node) : ComponentInterface(
      std::make_shared<rclcpp::node_interfaces::NodeInterfaces<ALL_RCLCPP_NODE_INTERFACES>>(
          node->get_node_base_interface(), node->get_node_clock_interface(), node->get_node_graph_interface(),
          node->get_node_logging_interface(), node->get_node_parameters_interface(),
          node->get_node_services_interface(), node->get_node_time_source_interface(),
          node->get_node_timers_interface(), node->get_node_topics_interface(),
          node->get_node_waitables_interface())) {}
  using ComponentInterface::node_base_;
  using ComponentInterface::node_clock_;
  using ComponentInterface::node_logging_;
  using ComponentInterface::node_parameters_;
  using ComponentInterface::add_parameter;
  using ComponentInterface::get_parameter;
  using ComponentInterface::get_parameter_value;
  using ComponentInterface::set_parameter_value;
  using ComponentInterface::parameter_map_;
  using ComponentInterface::add_predicate;
  using ComponentInterface::get_predicate;
  using ComponentInterface::set_predicate;
  using ComponentInterface::predicates_;
  using ComponentInterface::add_trigger;
  using ComponentInterface::trigger;
  using ComponentInterface::triggers_;
  using ComponentInterface::declare_input;
  using ComponentInterface::declare_output;
  using ComponentInterface::remove_input;
  using ComponentInterface::add_input;
  using ComponentInterface::add_service;
  using ComponentInterface::inputs_;
  using ComponentInterface::create_output;
  using ComponentInterface::outputs_;
  using ComponentInterface::periodic_outputs_;
  using ComponentInterface::empty_services_;
  using ComponentInterface::string_services_;
  using ComponentInterface::add_tf_broadcaster;
  using ComponentInterface::add_static_tf_broadcaster;
  using ComponentInterface::add_tf_listener;
  using ComponentInterface::publish_output;
  using ComponentInterface::send_transform;
  using ComponentInterface::send_transforms;
  using ComponentInterface::send_static_transform;
  using ComponentInterface::send_static_transforms;
  using ComponentInterface::lookup_transform;
  using ComponentInterface::raise_error;
  using ComponentInterface::get_qos;
  using ComponentInterface::set_qos;

  bool on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>&) override {
    validate_parameter_was_called = true;
    return validate_parameter_return_value;
  }

  rclcpp::Parameter get_ros_parameter(const std::string& name) {
    return this->node_parameters_->get_parameter(name);
  }

  rcl_interfaces::msg::SetParametersResult set_ros_parameter(const rclcpp::Parameter& parameter) {
    return this->node_parameters_->set_parameters({parameter}).at(
        0);
  }

  std::string get_parameter_description(const std::string& name) {
    return this->node_parameters_->describe_parameters({name}).at(0).description;
  }

  bool validate_parameter_was_called = false;
  bool validate_parameter_return_value = true;
};

class ComponentPublicInterface : public Component {
public:
  explicit ComponentPublicInterface(
      const rclcpp::NodeOptions& node_options, const std::string& fallback_name = "ComponentPublicInterface"
  ) : Component(node_options, fallback_name) {}
  using ComponentInterface::get_parameter_value;
  using ComponentInterface::get_rate;
  using ComponentInterface::get_period;
  using Component::add_output;
  using Component::outputs_;
  using Component::periodic_outputs_;
  using Component::remove_output;
  using Component::publish_output;
};

class LifecycleComponentPublicInterface : public LifecycleComponent {
public:
  explicit LifecycleComponentPublicInterface(const rclcpp::NodeOptions& node_options) :
      LifecycleComponent(node_options) {}
  using ComponentInterface::get_parameter_value;
  using ComponentInterface::get_rate;
  using ComponentInterface::get_period;
  using LifecycleComponent::add_output;
  using LifecycleComponent::configure_outputs;
  using LifecycleComponent::activate_outputs;
  using LifecycleComponent::outputs_;
  using LifecycleComponent::periodic_outputs_;
  using LifecycleComponent::remove_output;
  using LifecycleComponent::publish_output;
};
}// namespace modulo_components
