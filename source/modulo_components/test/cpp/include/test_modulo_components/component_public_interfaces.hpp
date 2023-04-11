#pragma once

#include "modulo_components/ComponentInterface.hpp"
#include "modulo_components/Component.hpp"
#include "modulo_components/LifecycleComponent.hpp"

using namespace state_representation;

namespace modulo_components {

template<class NodeT>
class ComponentInterfacePublicInterface : public ComponentInterface<NodeT> {
public:
  explicit ComponentInterfacePublicInterface(
      const rclcpp::NodeOptions& node_options, modulo_core::communication::PublisherType publisher_type,
      const std::string& fallback_name = "ComponentInterfacePublicInterface"
  ) : ComponentInterface<NodeT>(node_options, publisher_type, fallback_name) {}
  using ComponentInterface<NodeT>::add_parameter;
  using ComponentInterface<NodeT>::get_parameter;
  using ComponentInterface<NodeT>::get_parameter_value;
  using ComponentInterface<NodeT>::set_parameter_value;
  using ComponentInterface<NodeT>::parameter_map_;
  using ComponentInterface<NodeT>::add_predicate;
  using ComponentInterface<NodeT>::get_predicate;
  using ComponentInterface<NodeT>::set_predicate;
  using ComponentInterface<NodeT>::predicates_;
  using ComponentInterface<NodeT>::add_trigger;
  using ComponentInterface<NodeT>::trigger;
  using ComponentInterface<NodeT>::triggers_;
  using ComponentInterface<NodeT>::declare_input;
  using ComponentInterface<NodeT>::declare_output;
  using ComponentInterface<NodeT>::remove_input;
  using ComponentInterface<NodeT>::add_input;
  using ComponentInterface<NodeT>::add_service;
  using ComponentInterface<NodeT>::inputs_;
  using ComponentInterface<NodeT>::create_output;
  using ComponentInterface<NodeT>::outputs_;
  using ComponentInterface<NodeT>::periodic_outputs_;
  using ComponentInterface<NodeT>::empty_services_;
  using ComponentInterface<NodeT>::string_services_;
  using ComponentInterface<NodeT>::add_tf_broadcaster;
  using ComponentInterface<NodeT>::add_static_tf_broadcaster;
  using ComponentInterface<NodeT>::add_tf_listener;
  using ComponentInterface<NodeT>::publish_output;
  using ComponentInterface<NodeT>::send_transform;
  using ComponentInterface<NodeT>::send_transforms;
  using ComponentInterface<NodeT>::send_static_transform;
  using ComponentInterface<NodeT>::send_static_transforms;
  using ComponentInterface<NodeT>::lookup_transform;
  using ComponentInterface<NodeT>::raise_error;
  using ComponentInterface<NodeT>::get_qos;
  using ComponentInterface<NodeT>::set_qos;
  using ComponentInterface<NodeT>::get_clock;

  bool on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>&) override {
    validate_parameter_was_called = true;
    return validate_parameter_return_value;
  }

  rclcpp::Parameter get_ros_parameter(const std::string& name) {
    return NodeT::get_parameter(name);
  }

  rcl_interfaces::msg::SetParametersResult set_ros_parameter(const rclcpp::Parameter& parameter) {
    return NodeT::set_parameter(parameter);
  }

  std::string get_parameter_description(const std::string& name) {
    return NodeT::describe_parameter(name).description;
  }

  bool validate_parameter_was_called = false;
  bool validate_parameter_return_value = true;
};

class ComponentPublicInterface : public Component {
public:
  explicit ComponentPublicInterface(
      const rclcpp::NodeOptions& node_options, const std::string& fallback_name = "ComponentPublicInterface"
  ) : Component(node_options, fallback_name) {}
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
  using LifecycleComponent::add_output;
  using LifecycleComponent::configure_outputs;
  using LifecycleComponent::activate_outputs;
  using LifecycleComponent::outputs_;
  using LifecycleComponent::periodic_outputs_;
  using LifecycleComponent::remove_output;
  using LifecycleComponent::publish_output;
};
}// namespace modulo_components
