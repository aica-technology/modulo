#include "modulo_components/LifecycleComponent.hpp"

using namespace modulo_core::communication;
using namespace rclcpp_lifecycle;

namespace modulo_components {

LifecycleComponent::LifecycleComponent(const rclcpp::NodeOptions& node_options, const std::string& fallback_name)
    : ComponentInterface<rclcpp_lifecycle::LifecycleNode>(
        node_options, PublisherType::LIFECYCLE_PUBLISHER, fallback_name),
      has_error_(false) {}

template<>
double LifecycleComponent::get_period() const {
  return 1.0 / this->get_rate();
}

template<>
std::chrono::nanoseconds LifecycleComponent::get_period() const {
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 * this->get_period<double>()));
}

template<>
rclcpp::Duration LifecycleComponent::get_period() const {
  return rclcpp::Duration::from_seconds(this->get_period<double>());
}

void LifecycleComponent::step() {
  try {
    if (this->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      this->evaluate_periodic_callbacks();
      this->on_step_callback();
      this->publish_outputs();
    }
    this->publish_predicates();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to execute step function: " << ex.what());
    this->raise_error();
  }
}

node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleComponent::on_configure(const State& previous_state) {
  RCLCPP_DEBUG(this->get_logger(), "on_configure called from previous state %s", previous_state.label().c_str());
  if (previous_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    RCLCPP_WARN(get_logger(), "Invalid transition 'configure' from state %s.", previous_state.label().c_str());
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  if (!this->handle_configure()) {
    RCLCPP_WARN(get_logger(), "Configuration failed! Reverting to the unconfigured state.");
    // perform cleanup actions to ensure the component is unconfigured
    if (this->handle_cleanup()) {
      return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    } else {
      RCLCPP_ERROR(
          get_logger(),
          "Could not revert to the unconfigured state! Entering into the error processing transition state.");
      return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool LifecycleComponent::handle_configure() {
  bool result;
  try {
    result = this->on_configure_callback();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
    return false;
  }
  return result && this->configure_outputs();
}

bool LifecycleComponent::on_configure_callback() {
  return true;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleComponent::on_cleanup(const State& previous_state) {
  RCLCPP_DEBUG(this->get_logger(), "on_cleanup called from previous state %s", previous_state.label().c_str());
  if (previous_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_WARN(get_logger(), "Invalid transition 'cleanup' from state %s.", previous_state.label().c_str());
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  if (!this->handle_cleanup()) {
    RCLCPP_ERROR(get_logger(), "Cleanup failed! Entering into the error processing transition state.");
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool LifecycleComponent::handle_cleanup() {
  try {
    return this->on_cleanup_callback();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
    return false;
  }
}

bool LifecycleComponent::on_cleanup_callback() {
  return true;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleComponent::on_activate(const State& previous_state) {
  RCLCPP_DEBUG(this->get_logger(), "on_activate called from previous state %s", previous_state.label().c_str());
  if (previous_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_WARN(get_logger(), "Invalid transition 'activate' from state %s.", previous_state.label().c_str());
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  if (!this->handle_activate()) {
    RCLCPP_WARN(get_logger(), "Activation failed! Reverting to the inactive state.");
    // perform deactivation actions to ensure the component is inactive
    if (this->handle_deactivate()) {
      return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    } else {
      RCLCPP_ERROR(
          get_logger(), "Could not revert to the inactive state! Entering into the error processing transition state.");
      return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool LifecycleComponent::handle_activate() {
  bool result;
  try {
    result = this->on_activate_callback();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
    return false;
  }
  return result && this->activate_outputs();
}

bool LifecycleComponent::on_activate_callback() {
  return true;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleComponent::on_deactivate(const State& previous_state) {
  RCLCPP_DEBUG(this->get_logger(), "on_deactivate called from previous state %s", previous_state.label().c_str());
  if (previous_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(get_logger(), "Invalid transition 'deactivate' from state %s.", previous_state.label().c_str());
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  if (!this->handle_deactivate()) {
    RCLCPP_ERROR(get_logger(), "Deactivation failed! Entering into the error processing transition state.");
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool LifecycleComponent::handle_deactivate() {
  auto result = this->deactivate_outputs();
  try {
    return result && this->on_deactivate_callback();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
    return false;
  }
}

bool LifecycleComponent::on_deactivate_callback() {
  return true;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleComponent::on_shutdown(const State& previous_state) {
  RCLCPP_DEBUG(this->get_logger(), "on_shutdown called from previous state %s", previous_state.label().c_str());
  if (!this->has_error_) {
    switch (previous_state.id()) {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        if (!this->handle_deactivate()) {
          RCLCPP_DEBUG(get_logger(), "Shutdown failed during intermediate deactivation!");
          break;
        }
        [[fallthrough]];
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        if (!this->handle_cleanup()) {
          RCLCPP_DEBUG(get_logger(), "Shutdown failed during intermediate cleanup!");
          break;
        }
        [[fallthrough]];
      case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
        if (!this->handle_shutdown()) {
          break;
        }
        this->finalize_interfaces();
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
      default:
        RCLCPP_WARN(get_logger(), "Invalid transition 'shutdown' from state %s.", previous_state.label().c_str());
        break;
    }
  }
  RCLCPP_ERROR(get_logger(), "Entering into the error processing transition state.");
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
}

bool LifecycleComponent::handle_shutdown() {
  bool result;
  try {
    result = this->on_shutdown_callback();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
    return false;
  }
  return result;
}

bool LifecycleComponent::on_shutdown_callback() {
  return true;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleComponent::on_error(const State& previous_state) {
  RCLCPP_DEBUG(this->get_logger(), "on_error called from previous state %s", previous_state.label().c_str());
  bool error_handled;
  try {
    error_handled = this->handle_error();
  } catch (const std::exception& ex) {
    RCLCPP_DEBUG(this->get_logger(), "Exception caught during on_error handling: %s", ex.what());
    error_handled = false;
  }
  if (!error_handled) {
    RCLCPP_ERROR(get_logger(), "Error processing failed! Entering into the finalized state.");
    this->finalize_interfaces();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  this->has_error_ = false;
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool LifecycleComponent::handle_error() {
  return this->on_error_callback();
}

bool LifecycleComponent::on_error_callback() {
  return false;
}

bool LifecycleComponent::configure_outputs() {
  bool success = true;
  for (auto& [name, interface] : this->outputs_) {
    try {
      auto topic_name = this->get_parameter_value<std::string>(name + "_topic");
      RCLCPP_DEBUG_STREAM(
          this->get_logger(), "Configuring output '" << name << "' with topic name '" << topic_name << "'.");
      auto message_pair = interface->get_message_pair();
      switch (message_pair->get_type()) {
        case MessageType::BOOL: {
          auto publisher = this->create_publisher<std_msgs::msg::Bool>(topic_name, this->get_qos());
          interface =
              std::make_shared<
                  PublisherHandler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>, std_msgs::msg::Bool>>(
                  PublisherType::LIFECYCLE_PUBLISHER, publisher)
                  ->create_publisher_interface(message_pair);
          break;
        }
        case MessageType::FLOAT64: {
          auto publisher = this->create_publisher<std_msgs::msg::Float64>(topic_name, this->get_qos());
          interface = std::make_shared<PublisherHandler<
              rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>, std_msgs::msg::Float64>>(
                          PublisherType::LIFECYCLE_PUBLISHER, publisher)
                          ->create_publisher_interface(message_pair);
          break;
        }
        case MessageType::FLOAT64_MULTI_ARRAY: {
          auto publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, this->get_qos());
          interface = std::make_shared<PublisherHandler<
              rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>,
              std_msgs::msg::Float64MultiArray>>(PublisherType::LIFECYCLE_PUBLISHER, publisher)
                          ->create_publisher_interface(message_pair);
          break;
        }
        case MessageType::INT32: {
          auto publisher = this->create_publisher<std_msgs::msg::Int32>(topic_name, this->get_qos());
          interface =
              std::make_shared<
                  PublisherHandler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>, std_msgs::msg::Int32>>(
                  PublisherType::LIFECYCLE_PUBLISHER, publisher)
                  ->create_publisher_interface(message_pair);
          break;
        }
        case MessageType::STRING: {
          auto publisher = this->create_publisher<std_msgs::msg::String>(topic_name, this->get_qos());
          interface =
              std::make_shared<
                  PublisherHandler<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>, std_msgs::msg::String>>(
                  PublisherType::LIFECYCLE_PUBLISHER, publisher)
                  ->create_publisher_interface(message_pair);
          break;
        }
        case MessageType::ENCODED_STATE: {
          auto publisher = this->create_publisher<modulo_core::EncodedState>(topic_name, this->get_qos());
          interface = std::make_shared<PublisherHandler<
              rclcpp_lifecycle::LifecyclePublisher<modulo_core::EncodedState>, modulo_core::EncodedState>>(
                          PublisherType::LIFECYCLE_PUBLISHER, publisher)
                          ->create_publisher_interface(message_pair);
          break;
        }
        case MessageType::CUSTOM_MESSAGE: {
          interface = this->custom_output_configuration_callables_.at(name)(topic_name);
          break;
        }
      }
    } catch (const modulo_core::exceptions::CoreException& ex) {
      success = false;
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to configure output '" << name << "': " << ex.what());
    }
  }
  return success;
}

bool LifecycleComponent::activate_outputs() {
  bool success = true;
  for (auto const& [name, interface] : this->outputs_) {
    try {
      interface->activate();
    } catch (const modulo_core::exceptions::CoreException& ex) {
      success = false;
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to activate output '" << name << "': " << ex.what());
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "All outputs activated.");
  return success;
}

bool LifecycleComponent::deactivate_outputs() {
  bool success = true;
  for (auto const& [name, interface] : this->outputs_) {
    try {
      interface->deactivate();
    } catch (const modulo_core::exceptions::CoreException& ex) {
      success = false;
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to deactivate output '" << name << "': " << ex.what());
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "All outputs deactivated.");
  return success;
}

rclcpp_lifecycle::State LifecycleComponent::get_lifecycle_state() {
  return this->get_current_state();
}

void LifecycleComponent::raise_error() {
  ComponentInterface::raise_error();
  this->has_error_ = true;
  if (this->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  } else if (this->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  } else if (this->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
}
}// namespace modulo_components