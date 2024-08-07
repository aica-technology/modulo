#include "modulo_controllers/ControllerInterface.hpp"

#include <chrono>

#include <lifecycle_msgs/msg/state.hpp>

#include <modulo_core/translators/message_readers.hpp>

template<class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template<class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

using namespace modulo_core;
using namespace state_representation;
using namespace std::chrono_literals;

namespace modulo_controllers {

ControllerInterface::ControllerInterface(bool claim_all_state_interfaces)
    : controller_interface::ControllerInterface(),
      input_validity_period_(1.0),
      claim_all_state_interfaces_(claim_all_state_interfaces),
      on_init_called_(false) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ControllerInterface::on_init() {
  on_init_called_ = true;
  // registering set_parameter callbacks is only possible on_init since the lifecycle node is not yet initialized
  // on construction. This means we might not be able to validate parameter overrides - if they are provided.
  pre_set_parameter_cb_handle_ = get_node()->add_pre_set_parameters_callback(
      [this](std::vector<rclcpp::Parameter>& parameters) { return this->pre_set_parameters_callback(parameters); });
  on_set_parameter_cb_handle_ = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) -> rcl_interfaces::msg::SetParametersResult {
        return this->on_set_parameters_callback(parameters);
      });

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
        "input_validity_period", input_validity_period_,
        "The maximum age of an input state before discarding it as expired");
    add_parameter<int>("predicate_publishing_rate", 10, "The rate at which to publish controller predicates");

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
ControllerInterface::on_configure(const rclcpp_lifecycle::State&) {
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

  input_validity_period_ = get_parameter_value<double>("input_validity_period");
  add_inputs();
  add_outputs();

  if (predicates_.size()) {
    predicate_publisher_ =
        get_node()->create_publisher<modulo_interfaces::msg::PredicateCollection>("/predicates", qos_);
    predicate_message_.node = get_node()->get_fully_qualified_name();
    predicate_message_.type = modulo_interfaces::msg::PredicateCollection::CONTROLLER;

    predicate_timer_ = get_node()->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int64_t>(1e9 / get_parameter_value<int>("predicate_publishing_rate"))),
        [this]() { this->publish_predicates(); });
  }

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
  if (command_mutex_.try_lock()) {
    missed_locks_ = 0;
    ret = write_command_interfaces(period);
    command_mutex_.unlock();
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

void ControllerInterface::add_parameter(
    const std::shared_ptr<ParameterInterface>& parameter, const std::string& description, bool read_only) {
  rclcpp::Parameter ros_param;
  try {
    ros_param = translators::write_parameter(parameter);
  } catch (const modulo_core::exceptions::ParameterTranslationException& ex) {
    throw modulo_core::exceptions::ParameterException("Failed to add parameter: " + std::string(ex.what()));
  }
  if (!get_node()->has_parameter(parameter->get_name())) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Adding parameter '%s'.", parameter->get_name().c_str());
    parameter_map_.set_parameter(parameter);
    read_only_parameters_.insert_or_assign(parameter->get_name(), false);
    try {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description = description;
      descriptor.read_only = read_only;
      // since the pre_set_parameters_callback is not called on parameter declaration, this has to be true
      set_parameters_result_.successful = true;
      set_parameters_result_.reason = "";
      if (parameter->is_empty()) {
        descriptor.dynamic_typing = true;
        descriptor.type = translators::get_ros_parameter_type(parameter->get_parameter_type());
        get_node()->declare_parameter(parameter->get_name(), rclcpp::ParameterValue{}, descriptor);
      } else {
        get_node()->declare_parameter(parameter->get_name(), ros_param.get_parameter_value(), descriptor);
      }
      std::vector<rclcpp::Parameter> ros_parameters{get_node()->get_parameters({parameter->get_name()})};
      pre_set_parameters_callback(ros_parameters);
      auto result = on_set_parameters_callback(ros_parameters);
      if (!result.successful) {
        get_node()->undeclare_parameter(parameter->get_name());
        throw modulo_core::exceptions::ParameterException(result.reason);
      }
      read_only_parameters_.at(parameter->get_name()) = read_only;
    } catch (const std::exception& ex) {
      parameter_map_.remove_parameter(parameter->get_name());
      read_only_parameters_.erase(parameter->get_name());
      throw modulo_core::exceptions::ParameterException("Failed to add parameter: " + std::string(ex.what()));
    }
  } else {
    RCLCPP_DEBUG(get_node()->get_logger(), "Parameter '%s' already exists.", parameter->get_name().c_str());
  }
}

std::shared_ptr<ParameterInterface> ControllerInterface::get_parameter(const std::string& name) const {
  try {
    return parameter_map_.get_parameter(name);
  } catch (const state_representation::exceptions::InvalidParameterException& ex) {
    throw modulo_core::exceptions::ParameterException("Failed to get parameter '" + name + "': " + ex.what());
  }
}

void ControllerInterface::pre_set_parameters_callback(std::vector<rclcpp::Parameter>& parameters) {
  if (pre_set_parameter_callback_called_) {
    pre_set_parameter_callback_called_ = false;
    return;
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (auto& ros_parameter : parameters) {
    try {
      auto parameter = parameter_map_.get_parameter(ros_parameter.get_name());
      if (read_only_parameters_.at(ros_parameter.get_name())) {
        RCLCPP_DEBUG(get_node()->get_logger(), "Parameter '%s' is read only.", ros_parameter.get_name().c_str());
        continue;
      }

      // convert the ROS parameter into a ParameterInterface without modifying the original
      auto new_parameter = translators::read_parameter_const(ros_parameter, parameter);
      if (!this->validate_parameter(new_parameter)) {
        result.successful = false;
        result.reason += "Validation of parameter '" + ros_parameter.get_name() + "' returned false!";
      } else if (!new_parameter->is_empty()) {
        // update the value of the parameter in the map
        translators::copy_parameter_value(new_parameter, parameter);
        ros_parameter = translators::write_parameter(new_parameter);
      }
    } catch (const std::exception& ex) {
      result.successful = false;
      result.reason += ex.what();
    }
  }
  set_parameters_result_ = result;
}

rcl_interfaces::msg::SetParametersResult
ControllerInterface::on_set_parameters_callback(const std::vector<rclcpp::Parameter>&) {
  auto result = set_parameters_result_;
  set_parameters_result_.successful = true;
  set_parameters_result_.reason = "";
  return result;
}

bool ControllerInterface::validate_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  if (parameter->get_name() == "activation_timeout" || parameter->get_name() == "input_validity_period") {
    auto value = parameter->get_parameter_value<double>();
    if (value < 0.0 || value > std::numeric_limits<double>::max()) {
      RCLCPP_ERROR(
          get_node()->get_logger(), "Parameter value of parameter '%s' should be a positive finite number",
          parameter->get_name().c_str());
      return false;
    }
  }
  return on_validate_parameter_callback(parameter);
}

bool ControllerInterface::on_validate_parameter_callback(const std::shared_ptr<ParameterInterface>&) {
  return true;
}

void ControllerInterface::add_predicate(const std::string& predicate_name, bool predicate_value) {
  add_predicate(predicate_name, [predicate_value]() { return predicate_value; });
}

void ControllerInterface::add_predicate(
    const std::string& predicate_name, const std::function<bool(void)>& predicate_function) {
  if (predicate_name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to add predicate: Provide a non empty string as a name.");
    return;
  }
  if (predicates_.find(predicate_name) != predicates_.end()) {
    RCLCPP_WARN(
        get_node()->get_logger(), "Predicate with name '%s' already exists, overwriting.", predicate_name.c_str());
  } else {
    RCLCPP_DEBUG(get_node()->get_logger(), "Adding predicate '%s'.", predicate_name.c_str());
  }
  try {
    this->predicates_.insert_or_assign(predicate_name, modulo_core::Predicate(predicate_function));
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to evaluate callback of predicate '%s', returning false: %s",
        predicate_name.c_str(), ex.what());
  }
}

bool ControllerInterface::get_predicate(const std::string& predicate_name) const {
  auto predicate_it = predicates_.find(predicate_name);
  if (predicate_it == predicates_.end()) {
    RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to get predicate '%s': Predicate does not exists, returning false.", predicate_name.c_str());
    return false;
  }
  try {
    return predicate_it->second.get_value();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to evaluate callback of predicate '%s', returning false: %s", predicate_name.c_str(), ex.what());
  }
  return false;
}

void ControllerInterface::set_predicate(const std::string& predicate_name, bool predicate_value) {
  set_predicate(predicate_name, [predicate_value]() { return predicate_value; });
}

void ControllerInterface::set_predicate(
    const std::string& predicate_name, const std::function<bool(void)>& predicate_function) {
  auto predicate_it = predicates_.find(predicate_name);
  if (predicate_it == predicates_.end()) {
    RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to set predicate '%s': Predicate does not exist.", predicate_name.c_str());
    return;
  }
  predicate_it->second.set_predicate(predicate_function);
  if (auto new_predicate = predicate_it->second.query(); new_predicate) {
    publish_predicate(predicate_name, *new_predicate);
  }
}

void ControllerInterface::add_trigger(const std::string& trigger_name) {
  if (trigger_name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to add trigger: Provide a non empty string as a name.");
    return;
  }
  if (std::find(triggers_.cbegin(), triggers_.cend(), trigger_name) != triggers_.cend()) {
    RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to add trigger: there is already a trigger with name '%s'.",
        trigger_name.c_str());
    return;
  }
  if (predicates_.find(trigger_name) != predicates_.end()) {
    RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to add trigger: there is already a predicate with name '%s'.",
        trigger_name.c_str());
    return;
  }
  this->triggers_.push_back(trigger_name);
  this->add_predicate(trigger_name, false);
}

void ControllerInterface::trigger(const std::string& trigger_name) {
  if (std::find(triggers_.cbegin(), triggers_.cend(), trigger_name) != triggers_.cend()) {
    RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to trigger: could not find trigger with name  '%s'.", trigger_name.c_str());
    return;
  }
  this->set_predicate(trigger_name, true);
  // reset the trigger to be published on the next step
  this->predicates_.at(trigger_name).set_predicate([]() { return false; });
}

modulo_interfaces::msg::Predicate
ControllerInterface::get_predicate_message(const std::string& name, bool value) const {
  modulo_interfaces::msg::Predicate message;
  message.predicate = name;
  message.value = value;
  return message;
}

void ControllerInterface::publish_predicate(const std::string& predicate_name, bool value) const {
  auto message(predicate_message_);
  message.predicates.push_back(get_predicate_message(predicate_name, value));
  predicate_publisher_->publish(message);
}

void ControllerInterface::publish_predicates() {
  auto message(predicate_message_);
  for (auto predicate_it = predicates_.begin(); predicate_it != predicates_.end(); ++predicate_it) {
    if (auto new_predicate = predicate_it->second.query(); new_predicate) {
      message.predicates.push_back(get_predicate_message(predicate_it->first, *new_predicate));
    }
  }
  if (message.predicates.size()) {
    predicate_publisher_->publish(message);
  }
}

std::string ControllerInterface::validate_and_declare_signal(
    const std::string& signal_name, const std::string& type, const std::string& default_topic, bool fixed_topic) {
  auto parsed_signal_name = modulo_utils::parsing::parse_topic_name(signal_name);
  if (parsed_signal_name.empty()) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "The parsed signal name for %s '%s' is empty. Provide a string with valid characters for the signal name "
        "([a-zA-Z0-9_]).",
        type.c_str(), signal_name.c_str());
    return "";
  }
  if (signal_name != parsed_signal_name) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "The parsed signal name for %s '%s' is '%s'. Use the parsed signal name to refer to this %s and its topic "
        "parameter.",
        type.c_str(), signal_name.c_str(), parsed_signal_name.c_str(), type.c_str());
  }
  if (inputs_.find(parsed_signal_name) != inputs_.end()) {
    RCLCPP_WARN(get_node()->get_logger(), "Signal '%s' already exists as input.", parsed_signal_name.c_str());
    return "";
  }
  if (outputs_.find(parsed_signal_name) != outputs_.end()) {
    RCLCPP_WARN(get_node()->get_logger(), "Signal '%s' already exists as output", parsed_signal_name.c_str());
    return "";
  }
  auto topic = default_topic.empty() ? "~/" + parsed_signal_name : default_topic;
  auto parameter_name = parsed_signal_name + "_topic";
  if (get_node()->has_parameter(parameter_name) && get_parameter(parameter_name)->is_empty()) {
    set_parameter_value<std::string>(parameter_name, topic);
  } else {
    add_parameter<std::string>(
        parameter_name, topic, "Signal topic name of " + type + " '" + parsed_signal_name + "'", fixed_topic);
  }
  RCLCPP_DEBUG(
      get_node()->get_logger(), "Declared %s '%s' and parameter '%s' with value '%s'.", type.c_str(),
      parsed_signal_name.c_str(), parameter_name.c_str(), topic.c_str());
  return parsed_signal_name;
}

void ControllerInterface::create_input(
    const ControllerInput& input, const std::string& name, const std::string& topic_name) {
  auto parsed_name = validate_and_declare_signal(name, "input", topic_name);
  if (!parsed_name.empty()) {
    inputs_.insert_or_assign(name, input);
  }
}

void ControllerInterface::add_inputs() {
  for (auto& [name, input] : inputs_) {
    try {
      auto topic = get_parameter_value<std::string>(name + "_topic");
      std::visit(
          overloaded{
              [&](const realtime_tools::RealtimeBuffer<std::shared_ptr<EncodedState>>&) {
                subscriptions_.push_back(create_subscription<EncodedState>(name, topic));
              },
              [&](const realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Bool>>&) {
                subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(name, topic));
              },
              [&](const realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>>&) {
                subscriptions_.push_back(create_subscription<std_msgs::msg::Float64>(name, topic));
              },
              [&](const realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>>&) {
                subscriptions_.push_back(create_subscription<std_msgs::msg::Float64MultiArray>(name, topic));
              },
              [&](const realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Int32>>&) {
                subscriptions_.push_back(create_subscription<std_msgs::msg::Int32>(name, topic));
              },
              [&](const realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::String>>&) {
                subscriptions_.push_back(create_subscription<std_msgs::msg::String>(name, topic));
              }},
          input.buffer);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to add input '%s': %s", name.c_str(), ex.what());
    }
  }
}

void ControllerInterface::create_output(
    const PublisherVariant& publishers, const std::string& name, const std::string& topic_name) {
  auto parsed_name = validate_and_declare_signal(name, "output", topic_name);
  if (!parsed_name.empty()) {
    outputs_.insert_or_assign(name, publishers);
  }
}

void ControllerInterface::add_outputs() {
  for (auto& [name, publishers] : outputs_) {
    try {
      auto topic = get_parameter_value<std::string>(name + "_topic");
      std::visit(
          overloaded{
              [&](EncodedStatePublishers& pub) {
                std::get<1>(pub) = get_node()->create_publisher<EncodedState>(topic, qos_);
                std::get<2>(pub) = std::make_shared<realtime_tools::RealtimePublisher<EncodedState>>(std::get<1>(pub));
              },
              [&](BoolPublishers& pub) {
                pub.first = get_node()->create_publisher<std_msgs::msg::Bool>(topic, qos_);
                pub.second = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(pub.first);
              },
              [&](DoublePublishers& pub) {
                pub.first = get_node()->create_publisher<std_msgs::msg::Float64>(topic, qos_);
                pub.second = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(pub.first);
              },
              [&](DoubleVecPublishers& pub) {
                pub.first = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(topic, qos_);
                pub.second =
                    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(pub.first);
              },
              [&](IntPublishers& pub) {
                pub.first = get_node()->create_publisher<std_msgs::msg::Int32>(topic, qos_);
                pub.second = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Int32>>(pub.first);
              },
              [&](StringPublishers& pub) {
                pub.first = get_node()->create_publisher<std_msgs::msg::String>(topic, qos_);
                pub.second = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(pub.first);
              }},
          publishers);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to add input '%s': %s", name.c_str(), ex.what());
    }
  }
}

bool ControllerInterface::check_input_valid(const std::string& name) const {
  if (inputs_.find(name) == inputs_.end()) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000, "Could not find input '%s'", name.c_str());
    return false;
  }
  if (static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now() - inputs_.at(name).timestamp)
                              .count())
          / 1e9
      >= input_validity_period_) {
    return false;
  }
  return true;
}

std::string ControllerInterface::validate_service_name(const std::string& service_name, const std::string& type) const {
  std::string parsed_service_name = modulo_utils::parsing::parse_topic_name(service_name);
  if (parsed_service_name.empty()) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "The parsed service name for %s service '%s' is empty. Provide a string with valid characters for the service "
        "name "
        "([a-zA-Z0-9_]).",
        type.c_str(), service_name.c_str());
    return "";
  }
  if (service_name != parsed_service_name) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "The parsed name for '%s' service '%s' is '%s'. Use the parsed name to refer to this service.", type.c_str(),
        service_name.c_str(), parsed_service_name.c_str());
  }
  if (empty_services_.find(parsed_service_name) != empty_services_.cend()) {
    RCLCPP_WARN(
        get_node()->get_logger(), "Service with name '%s' already exists as empty service.",
        parsed_service_name.c_str());
    return "";
  }
  if (string_services_.find(parsed_service_name) != string_services_.cend()) {
    RCLCPP_WARN(
        get_node()->get_logger(), "Service with name '%s' already exists as string service.",
        parsed_service_name.c_str());
    return "";
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Adding %s service '%s'.", type.c_str(), parsed_service_name.c_str());
  return parsed_service_name;
}

void ControllerInterface::add_service(
    const std::string& service_name, const std::function<ControllerServiceResponse(void)>& callback) {
  auto parsed_service_name = validate_service_name(service_name, "empty");
  if (!parsed_service_name.empty()) {
    try {
      auto service = get_node()->create_service<modulo_interfaces::srv::EmptyTrigger>(
          "~/" + parsed_service_name,
          [this, callback](
              const std::shared_ptr<modulo_interfaces::srv::EmptyTrigger::Request>,
              std::shared_ptr<modulo_interfaces::srv::EmptyTrigger::Response> response) {
            try {
              if (this->command_mutex_.try_lock_for(100ms)) {
                auto callback_response = callback();
                this->command_mutex_.unlock();
                response->success = callback_response.success;
                response->message = callback_response.message;
              } else {
                response->success = false;
                response->message = "Unable to acquire lock for command interface within 100ms";
              }
            } catch (const std::exception& ex) {
              response->success = false;
              response->message = ex.what();
            }
          },
          qos_);
      empty_services_.insert_or_assign(parsed_service_name, service);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to add service '%s': %s", parsed_service_name.c_str(), ex.what());
    }
  }
}

void ControllerInterface::add_service(
    const std::string& service_name,
    const std::function<ControllerServiceResponse(const std::string& string)>& callback) {
  auto parsed_service_name = validate_service_name(service_name, "string");
  if (!parsed_service_name.empty()) {
    try {
      auto service = get_node()->create_service<modulo_interfaces::srv::StringTrigger>(
          "~/" + parsed_service_name,
          [this, callback](
              const std::shared_ptr<modulo_interfaces::srv::StringTrigger::Request> request,
              std::shared_ptr<modulo_interfaces::srv::StringTrigger::Response> response) {
            try {
              if (this->command_mutex_.try_lock_for(100ms)) {
                auto callback_response = callback(request->payload);
                this->command_mutex_.unlock();
                response->success = callback_response.success;
                response->message = callback_response.message;
              } else {
                response->success = false;
                response->message = "Unable to acquire lock for command interface within 100ms";
              }
            } catch (const std::exception& ex) {
              response->success = false;
              response->message = ex.what();
            }
          },
          qos_);
      string_services_.insert_or_assign(parsed_service_name, service);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to add service '%s': %s", parsed_service_name.c_str(), ex.what());
    }
  }
}

rclcpp::QoS ControllerInterface::get_qos() const {
  return qos_;
}

void ControllerInterface::set_qos(const rclcpp::QoS& qos) {
  qos_ = qos;
}

rclcpp_lifecycle::State ControllerInterface::get_lifecycle_state() const {
  return get_node()->get_current_state();
}

bool ControllerInterface::is_active() const {
  return get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

}// namespace modulo_controllers
