#include "modulo_components/Component.hpp"

using namespace modulo_core::communication;

namespace modulo_components {

Component::Component(const rclcpp::NodeOptions& node_options, const std::string& fallback_name) :
    ComponentInterface<rclcpp::Node>(node_options, PublisherType::PUBLISHER, fallback_name), started_(false) {
  this->add_predicate("is_finished", false);
}

void Component::step() {
  try {
    this->evaluate_periodic_callbacks();
    this->publish_outputs();
    this->publish_predicates();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to execute step function:" << ex.what());
    this->raise_error();
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
}// namespace modulo_components
