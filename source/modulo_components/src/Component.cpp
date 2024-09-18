#include "modulo_components/Component.hpp"

using namespace modulo_core::communication;
using namespace rclcpp;

namespace modulo_components {

Component::Component(const NodeOptions& node_options, const std::string& fallback_name)
    : Node(modulo_utils::parsing::parse_node_name(node_options, fallback_name), node_options),
      ComponentInterface(std::make_shared<node_interfaces::NodeInterfaces<ALL_RCLCPP_NODE_INTERFACES>>(
          Node::get_node_base_interface(), Node::get_node_clock_interface(), Node::get_node_graph_interface(),
          Node::get_node_logging_interface(), Node::get_node_parameters_interface(),
          Node::get_node_services_interface(), Node::get_node_time_source_interface(),
          Node::get_node_timers_interface(), Node::get_node_topics_interface(),
          Node::get_node_type_descriptions_interface(), Node::get_node_waitables_interface())),
      started_(false) {
  this->add_predicate("is_finished", false);
  this->add_predicate("in_error_state", false);
}

void Component::step() {
  if (!this->has_error()) {
    try {
      this->evaluate_periodic_callbacks();
      this->on_step_callback();
      this->publish_outputs();
      this->publish_predicates();
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to execute step function: " << ex.what());
      this->raise_error();
    }
  }
}

void Component::execute() {
  if (this->started_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start execution thread: Thread has already been started.");
    return;
  }
  this->started_ = true;
  this->execute_thread_ = std::thread([this]() { this->on_execute(); });
}

void Component::on_execute() {
  try {
    if (!this->on_execute_callback()) {
      this->raise_error();
      return;
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to run the execute function: " << ex.what());
    this->raise_error();
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Execution finished, setting 'is_finished' predicate to true.");
  this->set_predicate("is_finished", true);
}

bool Component::on_execute_callback() {
  return true;
}

std::shared_ptr<state_representation::ParameterInterface> Component::get_parameter(const std::string& name) const {
  return ComponentInterface::get_parameter(name);
}

void Component::raise_error() {
  ComponentInterface::raise_error();
  this->set_predicate("in_error_state", true);
}
}// namespace modulo_components
