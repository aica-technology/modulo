#pragma once

#include "modulo_components/Component.hpp"
#include "modulo_components/ComponentInterface.hpp"
#include "modulo_components/LifecycleComponent.hpp"

using namespace state_representation;

namespace modulo_components {

class ComponentInterfacePublicInterface : public ComponentInterface {
public:
  template<typename NodeT>
  explicit ComponentInterfacePublicInterface(const std::shared_ptr<NodeT>& node)
      : ComponentInterface(std::make_shared<rclcpp::node_interfaces::NodeInterfaces<ALL_RCLCPP_NODE_INTERFACES>>(
            node->get_node_base_interface(), node->get_node_clock_interface(), node->get_node_graph_interface(),
            node->get_node_logging_interface(), node->get_node_parameters_interface(),
            node->get_node_services_interface(), node->get_node_time_source_interface(),
            node->get_node_timers_interface(), node->get_node_topics_interface(),
            node->get_node_type_descriptions_interface(), node->get_node_waitables_interface())) {}
  using ComponentInterface::add_input;
  using ComponentInterface::add_parameter;
  using ComponentInterface::add_predicate;
  using ComponentInterface::add_assignment;
  using ComponentInterface::add_service;
  using ComponentInterface::add_static_tf_broadcaster;
  using ComponentInterface::add_tf_broadcaster;
  using ComponentInterface::add_tf_listener;
  using ComponentInterface::add_trigger;
  using ComponentInterface::create_output;
  using ComponentInterface::declare_input;
  using ComponentInterface::declare_output;
  using ComponentInterface::empty_services_;
  using ComponentInterface::get_parameter;
  using ComponentInterface::get_parameter_value;
  using ComponentInterface::get_predicate;
  using ComponentInterface::get_qos;
  using ComponentInterface::inputs_;
  using ComponentInterface::lookup_transform;
  using ComponentInterface::node_base_;
  using ComponentInterface::node_clock_;
  using ComponentInterface::node_logging_;
  using ComponentInterface::node_parameters_;
  using ComponentInterface::outputs_;
  using ComponentInterface::parameter_map_;
  using ComponentInterface::periodic_outputs_;
  using ComponentInterface::predicates_;
  using ComponentInterface::assignments_map_;
  using ComponentInterface::publish_output;
  using ComponentInterface::raise_error;
  using ComponentInterface::remove_input;
  using ComponentInterface::send_static_transform;
  using ComponentInterface::send_static_transforms;
  using ComponentInterface::send_transform;
  using ComponentInterface::send_transforms;
  using ComponentInterface::set_parameter_value;
  using ComponentInterface::set_predicate;
  using ComponentInterface::set_qos;
  using ComponentInterface::string_services_;
  using ComponentInterface::trigger;
  using ComponentInterface::trigger_assignment;
  using ComponentInterface::triggers_;

  bool on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>&) override {
    validate_parameter_was_called = true;
    return validate_parameter_return_value;
  }

  rclcpp::Parameter get_ros_parameter(const std::string& name) { return this->node_parameters_->get_parameter(name); }

  rcl_interfaces::msg::SetParametersResult set_ros_parameter(const rclcpp::Parameter& parameter) {
    return this->node_parameters_->set_parameters({parameter}).at(0);
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
      const rclcpp::NodeOptions& node_options, const std::string& fallback_name = "ComponentPublicInterface")
      : Component(node_options, fallback_name) {}
  using Component::add_output;
  using Component::outputs_;
  using Component::periodic_outputs_;
  using Component::publish_output;
  using Component::remove_output;
  using ComponentInterface::get_parameter_value;
  using ComponentInterface::get_period;
  using ComponentInterface::get_predicate;
  using ComponentInterface::get_rate;
};

class LifecycleComponentPublicInterface : public LifecycleComponent {
public:
  explicit LifecycleComponentPublicInterface(
      const rclcpp::NodeOptions& node_options, const std::string& fallback_name = "LifecycleComponentPublicInterface")
      : LifecycleComponent(node_options, fallback_name) {}
  using ComponentInterface::get_parameter_value;
  using ComponentInterface::get_period;
  using ComponentInterface::get_predicate;
  using ComponentInterface::get_rate;
  using LifecycleComponent::activate_outputs;
  using LifecycleComponent::add_output;
  using LifecycleComponent::configure_outputs;
  using LifecycleComponent::outputs_;
  using LifecycleComponent::periodic_outputs_;
  using LifecycleComponent::publish_output;
  using LifecycleComponent::remove_output;
};
}// namespace modulo_components
