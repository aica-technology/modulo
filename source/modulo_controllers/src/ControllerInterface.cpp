#include "modulo_controllers/ControllerInterface.hpp"

using namespace modulo_core;
using namespace state_representation;
using namespace std::chrono_literals;

namespace modulo_controllers {

ControllerInterface::ControllerInterface(bool claim_all_state_interfaces)
    : BaseControllerInterface(), claim_all_state_interfaces_(claim_all_state_interfaces), on_init_called_(false) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ControllerInterface::on_init() {
  auto status = BaseControllerInterface::on_init();
  if (status != CallbackReturn::SUCCESS) {
    return status;
  }
  on_init_called_ = true;

  try {
    add_parameter(std::make_shared<Parameter<std::string>>("hardware_name"), "The name of the hardware interface");
    add_parameter(
        std::make_shared<Parameter<std::string>>("robot_description"),
        "The string formatted content of the controller's URDF description");
    add_parameter(
        std::make_shared<Parameter<std::vector<std::string>>>("joints"),
        "A vector of joint names that the controller will claim");
    add_parameter<double>(
        "activation_timeout", 1.0, "The seconds to wait for valid data on the state interfaces before activating");
    add_parameter<double>(
        "input_validity_period", 1.0, "The maximum age of an input state before discarding it as expired");
    add_parameter<double>("predicate_publishing_rate", 10.0, "The rate at which to publish controller predicates");

    return add_interfaces();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during on_init stage with message: %s \n", e.what());
  }
  return CallbackReturn::ERROR;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ControllerInterface::add_interfaces() {
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ControllerInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
  auto status = BaseControllerInterface::on_configure(previous_state);
  if (status != CallbackReturn::SUCCESS) {
    return status;
  }

  if (!on_init_called_) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "The controller has not been properly initialized! Derived controller classes must call "
        "'ControllerInterface::on_init()' during their own initialization before being configured.");
    return CallbackReturn::ERROR;
  }

  auto hardware_name = get_parameter("hardware_name");
  if (hardware_name->is_empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'hardware_name' cannot be empty");
    return CallbackReturn::ERROR;
  }
  hardware_name_ = hardware_name->get_parameter_value<std::string>();

  set_input_validity_period(get_parameter_value<double>("input_validity_period"));

  RCLCPP_DEBUG(get_node()->get_logger(), "Configuration of ControllerInterface successful");
  try {
    return on_configure();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), ex.what());
  }
  return CallbackReturn::ERROR;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ControllerInterface::on_configure() {
  return CallbackReturn::SUCCESS;
}

void ControllerInterface::add_state_interface(const std::string& name, const std::string& interface) {
  add_interface(name, interface, state_interface_names_, "state");
}

void ControllerInterface::add_command_interface(const std::string& name, const std::string& interface) {
  add_interface(name, interface, command_interface_names_, "command");
}

void ControllerInterface::add_interface(
    const std::string& name, const std::string& interface, std::vector<std::string>& list, const std::string& type) {
  if (get_node()->get_current_state().label() != "configuring") {
    throw std::runtime_error("Interfaces can only be added when the controller is in state 'configuring'");
  }
  auto full_name = name + "/" + interface;
  if (std::find(list.cbegin(), list.cend(), full_name) == list.cend()) {
    list.push_back(full_name);
    RCLCPP_DEBUG(
        get_node()->get_logger(), "Adding interface '%s' to the list of desired %s interfaces", full_name.c_str(),
        type.c_str());
  } else {
    RCLCPP_WARN(
        get_node()->get_logger(), "Interface '%s' is already in the list of desired %s interfaces", full_name.c_str(),
        type.c_str());
  }
}

controller_interface::InterfaceConfiguration ControllerInterface::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  if (command_interface_names_.empty()) {
    RCLCPP_DEBUG(get_node()->get_logger(), "List of command interfaces is empty, not claiming any interfaces.");
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
  }

  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& interface : command_interface_names_) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Claiming command interface '%s'", interface.c_str());
    command_interfaces_config.names.push_back(interface);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ControllerInterface::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  if (claim_all_state_interfaces_) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Claiming all state interfaces.");
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
    return state_interfaces_config;
  }

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& interface : state_interface_names_) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Claiming state interface '%s'", interface.c_str());
    state_interfaces_config.names.push_back(interface);
  }

  return state_interfaces_config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ControllerInterface::on_activate(const rclcpp_lifecycle::State&) {
  // initialize the map of command data from all available interfaces
  command_interface_data_ = std::vector<double>(command_interfaces_.size());
  for (unsigned int i = 0; i < command_interfaces_.size(); ++i) {
    const auto& command_interface = command_interfaces_.at(i);
    if (command_interface_indices_.find(command_interface.get_prefix_name()) == command_interface_indices_.cend()) {
      command_interface_indices_.insert_or_assign(
          command_interface.get_prefix_name(), std::unordered_map<std::string, unsigned int>());
    }
    command_interface_indices_.at(command_interface.get_prefix_name())
        .insert_or_assign(command_interface.get_interface_name(), i);
    command_interface_data_.at(i) = command_interface.get_value();
  }

  // initialize the map of state data from all available interfaces
  for (const auto& state_interface : state_interfaces_) {
    if (state_interface_data_.find(state_interface.get_prefix_name()) == state_interface_data_.cend()) {
      state_interface_data_.insert_or_assign(
          state_interface.get_prefix_name(), std::unordered_map<std::string, double>());
    }
    state_interface_data_.at(state_interface.get_prefix_name())
        .insert_or_assign(state_interface.get_interface_name(), state_interface.get_value());
  }

  auto status = CallbackReturn::ERROR;
  try {
    status = on_activate();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), ex.what());
  }
  if (status != CallbackReturn::SUCCESS) {
    return status;
  }

  auto start_time = get_node()->get_clock()->now();
  auto activation_timeout = rclcpp::Duration::from_seconds(get_parameter_value<double>("activation_timeout"));
  while (read_state_interfaces() != controller_interface::return_type::OK) {
    RCLCPP_DEBUG_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Activation is not possible yet; the controller did not receive valid states from hardware");
    if ((get_node()->get_clock()->now() - start_time) > activation_timeout) {
      release_interfaces();
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Activation was not successful; the controller did not receive valid states from hardware");
      return CallbackReturn::FAILURE;
    }
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "Activation of ControllerInterface successful");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ControllerInterface::on_activate() {
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ControllerInterface::on_deactivate(const rclcpp_lifecycle::State&) {
  try {
    return on_deactivate();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), ex.what());
  }
  return CallbackReturn::ERROR;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ControllerInterface::on_deactivate() {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
ControllerInterface::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  auto status = read_state_interfaces();
  if (status != controller_interface::return_type::OK) {
    return status;
  }

  try {
    status = evaluate(time, period.to_chrono<std::chrono::nanoseconds>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000, "Exception during evaluate(): %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }
  if (status != controller_interface::return_type::OK) {
    return status;
  }

  if (command_interface_data_.empty()) {
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type ret;
  if (get_command_mutex().try_lock()) {
    missed_locks_ = 0;
    ret = write_command_interfaces(period);
    get_command_mutex().unlock();
  } else {
    if (missed_locks_ > 2) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Controller is unable to acquire lock for command interfaces, returning an error now");
      ret = controller_interface::return_type::ERROR;
    }
    ++missed_locks_;
    RCLCPP_WARN(get_node()->get_logger(), "Unable to acquire lock for command interfaces (%u/3)", missed_locks_);
  }

  return ret;
}

controller_interface::return_type ControllerInterface::read_state_interfaces() {
  for (const auto& state_interface : state_interfaces_) {
    state_interface_data_.at(state_interface.get_prefix_name()).at(state_interface.get_interface_name()) =
        state_interface.get_value();
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type ControllerInterface::write_command_interfaces(const rclcpp::Duration&) {
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(command_interface_data_.at(
        command_interface_indices_.at(command_interface.get_prefix_name()).at(command_interface.get_interface_name())));
  }
  return controller_interface::return_type::OK;
}

std::unordered_map<std::string, double> ControllerInterface::get_state_interfaces(const std::string& name) const {
  return state_interface_data_.at(name);
}

double ControllerInterface::get_state_interface(const std::string& name, const std::string& interface) const {
  return state_interface_data_.at(name).at(interface);
}

double ControllerInterface::get_command_interface(const std::string& name, const std::string& interface) const {
  return command_interfaces_.at(command_interface_indices_.at(name).at(interface)).get_value();
}

void ControllerInterface::set_command_interface(const std::string& name, const std::string& interface, double value) {
  try {
    command_interface_data_.at(command_interface_indices_.at(name).at(interface)) = value;
  } catch (const std::out_of_range&) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "set_command_interface called with an unknown name/interface: %s/%s", name.c_str(), interface.c_str());
  }
}

}// namespace modulo_controllers
