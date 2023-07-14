#include "modulo_components/ComponentInterface.hpp"

using namespace modulo_core::communication;

namespace modulo_components {

ComponentInterface::ComponentInterface(
    const std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<ALL_RCLCPP_NODE_INTERFACES>>& interfaces
) :
    node_base_(interfaces->get_node_base_interface()),
    node_clock_(interfaces->get_node_clock_interface()),
    node_logging_(interfaces->get_node_logging_interface()),
    node_parameters_(interfaces->get_node_parameters_interface()),
    node_services_(interfaces->get_node_services_interface()),
    node_timers_(interfaces->get_node_timers_interface()),
    node_topics_(interfaces->get_node_topics_interface()) {
  this->cb_group_ = node_base_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // register the parameter change callback handler
  this->parameter_cb_handle_ = this->node_parameters_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) -> rcl_interfaces::msg::SetParametersResult {
        return this->on_set_parameters_callback(parameters);
      });
  this->add_parameter("period", 0.1, "The time interval in seconds for all periodic callbacks", true);
  this->predicate_publisher_ = rclcpp::create_publisher<modulo_component_interfaces::msg::Predicate>(
      this->node_parameters_, this->node_topics_, "/predicates", this->qos_);

  this->add_predicate("in_error_state", false);

  this->step_timer_ = rclcpp::create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(this->get_parameter_value<double>("period") * 1e9)),
      [this] { this->step(); }, this->cb_group_, this->node_base_.get(), this->node_timers_.get());
}
}// namespace modulo_components
