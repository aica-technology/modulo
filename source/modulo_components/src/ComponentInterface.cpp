#include "modulo_components/ComponentInterface.hpp"

#include <console_bridge/console.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <modulo_core/exceptions/ParameterTranslationException.hpp>
#include <modulo_core/translators/message_readers.hpp>
#include <modulo_core/translators/message_writers.hpp>

#include "modulo_components/exceptions/AddServiceException.hpp"
#include "modulo_components/exceptions/LookupTransformException.hpp"

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
  // register the parameter change callback handler
  this->parameter_cb_handle_ = this->node_parameters_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) -> rcl_interfaces::msg::SetParametersResult {
        return this->on_set_parameters_callback(parameters);
      });
  this->add_parameter("rate", 10, "The rate in Hertz for all periodic callbacks", true);
  this->add_parameter("period", 0.1, "The time interval in seconds for all periodic callbacks", true);

  this->predicate_publisher_ = rclcpp::create_publisher<modulo_component_interfaces::msg::Predicate>(
      this->node_parameters_, this->node_topics_, "/predicates", this->qos_);

  this->add_predicate("in_error_state", false);

  this->step_timer_ = rclcpp::create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(this->get_parameter_value<double>("period") * 1e9)),
      [this] {
        if (this->step_mutex_.try_lock()) {
          this->step();
          this->step_mutex_.unlock();
        }
      },
      nullptr, this->node_base_.get(), this->node_timers_.get());
}

ComponentInterface::~ComponentInterface() {
  this->step_mutex_.lock();
}

void ComponentInterface::step() {}

void ComponentInterface::on_step_callback() {}

void ComponentInterface::add_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>& parameter, const std::string& description,
    bool read_only
) {
  this->set_parameter_callback_called_ = false;
  rclcpp::Parameter ros_param;
  try {
    ros_param = modulo_core::translators::write_parameter(parameter);
  } catch (const modulo_core::exceptions::ParameterTranslationException& ex) {
    throw exceptions::ComponentParameterException("Failed to add parameter: " + std::string(ex.what()));
  }
  if (!this->node_parameters_->has_parameter(parameter->get_name())) {
    RCLCPP_DEBUG_STREAM(this->node_logging_->get_logger(), "Adding parameter '" << parameter->get_name() << "'.");
    this->parameter_map_.set_parameter(parameter);
    try {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description = description;
      descriptor.read_only = read_only;
      if (parameter->is_empty()) {
        descriptor.dynamic_typing = true;
        descriptor.type = modulo_core::translators::get_ros_parameter_type(parameter->get_parameter_type());
        this->node_parameters_->declare_parameter(parameter->get_name(), rclcpp::ParameterValue{}, descriptor);
      } else {
        this->node_parameters_->declare_parameter(parameter->get_name(), ros_param.get_parameter_value(), descriptor);
      }
      if (!this->set_parameter_callback_called_) {
        auto result =
            this->on_set_parameters_callback({this->node_parameters_->get_parameters({parameter->get_name()})});
        if (!result.successful) {
          this->node_parameters_->undeclare_parameter(parameter->get_name());
          throw exceptions::ComponentParameterException(result.reason);
        }
      }
    } catch (const std::exception& ex) {
      this->parameter_map_.remove_parameter(parameter->get_name());
      throw exceptions::ComponentParameterException("Failed to add parameter: " + std::string(ex.what()));
    }
  } else {
    RCLCPP_DEBUG_STREAM(this->node_logging_->get_logger(),
                        "Parameter '" << parameter->get_name() << "' already exists.");
  }
}

std::shared_ptr<state_representation::ParameterInterface>
ComponentInterface::get_parameter(const std::string& name) const {
  try {
    return this->parameter_map_.get_parameter(name);
  } catch (const state_representation::exceptions::InvalidParameterException& ex) {
    throw exceptions::ComponentParameterException("Failed to get parameter '" + name + "': " + ex.what());
  }
}

rcl_interfaces::msg::SetParametersResult
ComponentInterface::on_set_parameters_callback(const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto& ros_parameter : parameters) {
    try {
      if (ros_parameter.get_name().substr(0, 27) == "qos_overrides./tf.publisher") {
        continue;
      }
      // get the associated parameter interface by name
      auto parameter = parameter_map_.get_parameter(ros_parameter.get_name());

      // convert the ROS parameter into a ParameterInterface without modifying the original
      auto new_parameter = modulo_core::translators::read_parameter_const(ros_parameter, parameter);
      if (!this->validate_parameter(new_parameter)) {
        result.successful = false;
        result.reason += "Validation of parameter '" + ros_parameter.get_name() + "' returned false!";
      } else if (!new_parameter->is_empty()) {
        // update the value of the parameter in the map
        modulo_core::translators::copy_parameter_value(new_parameter, parameter);
      }
    } catch (const std::exception& ex) {
      result.successful = false;
      result.reason += ex.what();
    }
  }
  this->set_parameter_callback_called_ = true;
  return result;
}

bool
ComponentInterface::validate_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) {
  if (parameter->get_name() == "rate") {
    auto value = parameter->get_parameter_value<int>();
    if (value <= 0 || !std::isfinite(value)) {
      RCLCPP_ERROR(this->node_logging_->get_logger(), "Value for parameter 'rate' has to be a positive finite number.");
      return false;
    }
  }
  if (parameter->get_name() == "period") {
    auto value = parameter->get_parameter_value<double>();
    if (value <= 0.0 || !std::isfinite(value)) {
      RCLCPP_ERROR(this->node_logging_->get_logger(),
                   "Value for parameter 'period' has to be a positive finite number.");
      return false;
    }
  }
  return this->on_validate_parameter_callback(parameter);
}

bool
ComponentInterface::on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>&) {
  return true;
}

void ComponentInterface::add_predicate(const std::string& name, bool predicate) {
  this->add_variant_predicate(name, modulo_utils::PredicateVariant(predicate));
}

void ComponentInterface::add_predicate(const std::string& name, const std::function<bool(void)>& predicate) {
  this->add_variant_predicate(name, modulo_utils::PredicateVariant(predicate));
}

void ComponentInterface::add_variant_predicate(
    const std::string& name, const modulo_utils::PredicateVariant& predicate) {
  if (name.empty()) {
    RCLCPP_ERROR(this->node_logging_->get_logger(), "Failed to add predicate: Provide a non empty string as a name.");
    return;
  }
  if (this->predicates_.find(name) != this->predicates_.end()) {
    RCLCPP_WARN_STREAM(this->node_logging_->get_logger(),
                       "Predicate with name '" << name << "' already exists, overwriting.");
  } else {
    RCLCPP_DEBUG_STREAM(this->node_logging_->get_logger(), "Adding predicate '" << name << "'.");
  }
  this->predicates_.insert_or_assign(name, predicate);
}

bool ComponentInterface::get_predicate(const std::string& predicate_name) {
  auto predicate_iterator = this->predicates_.find(predicate_name);
  // if there is no predicate with that name simply return false with an error message
  if (predicate_iterator == this->predicates_.end()) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->node_logging_->get_logger(), *this->node_clock_->get_clock(), 1000,
                                 "Failed to get predicate '" << predicate_name
                                                             << "': Predicate does not exists, returning false.");
    return false;
  }
  // try to get the value from the variant as a bool
  auto* ptr_value = std::get_if<bool>(&predicate_iterator->second);
  if (ptr_value) {
    return *ptr_value;
  }
  // if previous check failed, it means the variant is actually a callback function
  auto callback_function = std::get<std::function<bool(void)>>(predicate_iterator->second);
  bool value = false;
  try {
    value = (callback_function)();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->node_logging_->get_logger(), *this->node_clock_->get_clock(), 1000,
                                 "Failed to evaluate callback of predicate '" << predicate_name
                                                                              << "', returning false: " << ex.what());
  }
  return value;
}

void ComponentInterface::set_predicate(const std::string& name, bool predicate) {
  this->set_variant_predicate(name, modulo_utils::PredicateVariant(predicate));
}

void ComponentInterface::set_predicate(
    const std::string& name, const std::function<bool(void)>& predicate
) {
  this->set_variant_predicate(name, modulo_utils::PredicateVariant(predicate));
}

void ComponentInterface::set_variant_predicate(
    const std::string& name, const modulo_utils::PredicateVariant& predicate) {
  auto predicate_iterator = this->predicates_.find(name);
  if (predicate_iterator == this->predicates_.end()) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->node_logging_->get_logger(), *this->node_clock_->get_clock(), 1000,
                                 "Failed to set predicate '" << name << "': Predicate does not exist.");
    return;
  }
  predicate_iterator->second = predicate;
  this->publish_predicate(name);
}

void ComponentInterface::add_trigger(const std::string& trigger_name) {
  if (trigger_name.empty()) {
    RCLCPP_ERROR(this->node_logging_->get_logger(), "Failed to add trigger: Provide a non empty string as a name.");
    return;
  }
  if (this->triggers_.find(trigger_name) != this->triggers_.end()
      || this->predicates_.find(trigger_name) != this->predicates_.end()) {
    RCLCPP_ERROR_STREAM(this->node_logging_->get_logger(), "Failed to add trigger: there is already a trigger or "
                                                           "predicate with name '" << trigger_name << "'.");
    return;
  }
  this->triggers_.insert_or_assign(trigger_name, false);
  this->add_predicate(
      trigger_name, [this, trigger_name] {
        auto value = this->triggers_.at(trigger_name);
        this->triggers_.at(trigger_name) = false;
        return value;
      });
}

void ComponentInterface::trigger(const std::string& trigger_name) {
  if (this->triggers_.find(trigger_name) == this->triggers_.end()) {
    RCLCPP_ERROR_STREAM(this->node_logging_->get_logger(), "Failed to trigger: could not find trigger"
                                                           " with name  '" << trigger_name << "'.");
    return;
  }
  this->triggers_.at(trigger_name) = true;
  publish_predicate(trigger_name);
}

void ComponentInterface::declare_input(
    const std::string& signal_name, const std::string& default_topic, bool fixed_topic
) {
  this->declare_signal(signal_name, "input", default_topic, fixed_topic);
}

void ComponentInterface::declare_output(
    const std::string& signal_name, const std::string& default_topic, bool fixed_topic
) {
  this->declare_signal(signal_name, "output", default_topic, fixed_topic);
}

void ComponentInterface::declare_signal(
    const std::string& signal_name, const std::string& type, const std::string& default_topic, bool fixed_topic
) {
  std::string parsed_signal_name = modulo_utils::parse_topic_name(signal_name);
  if (parsed_signal_name.empty()) {
    throw exceptions::AddSignalException(
        "The parsed signal name for " + type + " '" + signal_name
            + "' is empty. Provide a string with valid characters for the signal name ([a-zA-Z0-9_]).");
  }
  if (signal_name != parsed_signal_name) {
    RCLCPP_WARN_STREAM(
        this->node_logging_->get_logger(),
        "The parsed signal name for " + type + " '" + signal_name + "' is '" + parsed_signal_name
            + "'. Use the parsed signal name to refer to this " + type + " and its topic parameter");
  }
  if (this->inputs_.find(parsed_signal_name) != this->inputs_.cend()) {
    throw exceptions::AddSignalException("Signal with name '" + parsed_signal_name + "' already exists as input.");
  }
  if (this->outputs_.find(parsed_signal_name) != this->outputs_.cend()) {
    throw exceptions::AddSignalException("Signal with name '" + parsed_signal_name + "' already exists as output.");
  }
  std::string topic_name = default_topic.empty() ? "~/" + parsed_signal_name : default_topic;
  auto parameter_name = parsed_signal_name + "_topic";
  if (this->node_parameters_->has_parameter(parameter_name) && this->get_parameter(parameter_name)->is_empty()) {
    this->set_parameter_value<std::string>(parameter_name, topic_name);
  } else {
    this->add_parameter(
        parameter_name, topic_name, "Signal topic name of " + type + " '" + parsed_signal_name + "'", fixed_topic);
  }
  RCLCPP_DEBUG_STREAM(this->node_logging_->get_logger(),
                      "Declared signal '" << parsed_signal_name << "' and parameter '" << parameter_name
                                          << "' with value '" << topic_name << "'.");
}

void ComponentInterface::publish_output(const std::string& signal_name) {
  auto parsed_signal_name = modulo_utils::parse_topic_name(signal_name);
  if (this->outputs_.find(parsed_signal_name) == this->outputs_.cend()) {
    throw exceptions::ComponentException("Output with name '" + signal_name + "' doesn't exist.");
  }
  if (this->periodic_outputs_.at(parsed_signal_name)) {
    throw exceptions::ComponentException("An output that is published periodically cannot be triggered manually.");
  }
  try {
    this->outputs_.at(parsed_signal_name)->publish();
  } catch (const modulo_core::exceptions::CoreException& ex) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->node_logging_->get_logger(), *this->node_clock_->get_clock(), 1000,
                                 "Failed to publish output '" << parsed_signal_name << "': " << ex.what());
  }
}

void ComponentInterface::remove_input(const std::string& signal_name) {
  if (!this->remove_signal(signal_name, this->inputs_)) {
    auto parsed_signal_name = modulo_utils::parse_topic_name(signal_name);
    if (!this->remove_signal(parsed_signal_name, this->inputs_)) {
      RCLCPP_DEBUG_STREAM(this->node_logging_->get_logger(),
                          "Unknown input '" << signal_name << "' (parsed name was '" << parsed_signal_name << "').");
    }
  }
}

void ComponentInterface::remove_output(const std::string& signal_name) {
  if (!this->remove_signal(signal_name, this->outputs_)) {
    auto parsed_signal_name = modulo_utils::parse_topic_name(signal_name);
    if (!this->remove_signal(parsed_signal_name, this->outputs_)) {
      RCLCPP_DEBUG_STREAM(this->node_logging_->get_logger(),
                          "Unknown output '" << signal_name << "' (parsed name was '" << parsed_signal_name << "').");
    }
  }
}

void ComponentInterface::add_service(
    const std::string& service_name, const std::function<ComponentServiceResponse(void)>& callback
) {
  try {
    std::string parsed_service_name = this->validate_service_name(service_name);
    RCLCPP_DEBUG_STREAM(this->node_logging_->get_logger(), "Adding empty service '" << parsed_service_name << "'.");
    auto service = rclcpp::create_service<modulo_component_interfaces::srv::EmptyTrigger>(
        this->node_base_, this->node_services_, "~/" + parsed_service_name, [callback](
            const std::shared_ptr<modulo_component_interfaces::srv::EmptyTrigger::Request>,
            std::shared_ptr<modulo_component_interfaces::srv::EmptyTrigger::Response> response
        ) {
          try {
            auto callback_response = callback();
            response->success = callback_response.success;
            response->message = callback_response.message;
          } catch (const std::exception& ex) {
            response->success = false;
            response->message = ex.what();
          }
        }, this->qos_, nullptr);
    this->empty_services_.insert_or_assign(parsed_service_name, service);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->node_logging_->get_logger(),
                        "Failed to add service '" << service_name << "': " << ex.what());
  }
}

void ComponentInterface::add_service(
    const std::string& service_name, const std::function<ComponentServiceResponse(const std::string& string)>& callback
) {
  try {
    std::string parsed_service_name = this->validate_service_name(service_name);
    RCLCPP_DEBUG_STREAM(this->node_logging_->get_logger(), "Adding string service '" << parsed_service_name << "'.");
    auto service = rclcpp::create_service<modulo_component_interfaces::srv::StringTrigger>(
        this->node_base_, this->node_services_, "~/" + parsed_service_name, [callback](
            const std::shared_ptr<modulo_component_interfaces::srv::StringTrigger::Request> request,
            std::shared_ptr<modulo_component_interfaces::srv::StringTrigger::Response> response
        ) {
          try {
            auto callback_response = callback(request->payload);
            response->success = callback_response.success;
            response->message = callback_response.message;
          } catch (const std::exception& ex) {
            response->success = false;
            response->message = ex.what();
          }
        }, this->qos_, nullptr);
    this->string_services_.insert_or_assign(parsed_service_name, service);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->node_logging_->get_logger(),
                        "Failed to add service '" << service_name << "': " << ex.what());
  }
}

std::string ComponentInterface::validate_service_name(const std::string& service_name) {
  std::string parsed_service_name = modulo_utils::parse_topic_name(service_name);
  if (parsed_service_name.empty()) {
    throw exceptions::AddServiceException(
        "The parsed service name for service '" + service_name
            + "' is empty. Provide a string with valid characters for the signal name ([a-zA-Z0-9_]).");
  }
  if (service_name != parsed_service_name) {
    RCLCPP_WARN_STREAM(
        this->node_logging_->get_logger(),
        "The parsed name for service '" + service_name + "' is '" + parsed_service_name
            + "'. Use the parsed name to refer to this service");
  }
  if (this->empty_services_.find(parsed_service_name) != this->empty_services_.cend()
      || this->string_services_.find(parsed_service_name) != this->string_services_.cend()) {
    throw exceptions::AddServiceException("Service with name '" + parsed_service_name + "' already exists.");
  }
  return parsed_service_name;
}

void ComponentInterface::add_periodic_callback(const std::string& name, const std::function<void()>& callback) {
  if (name.empty()) {
    RCLCPP_ERROR(this->node_logging_->get_logger(),
                 "Failed to add periodic function: Provide a non empty string as a name.");
    return;
  }
  if (this->periodic_callbacks_.find(name) != this->periodic_callbacks_.end()) {
    RCLCPP_WARN_STREAM(this->node_logging_->get_logger(),
                       "Periodic function '" << name << "' already exists, overwriting.");
  } else {
    RCLCPP_DEBUG_STREAM(this->node_logging_->get_logger(), "Adding periodic function '" << name << "'.");
  }
  this->periodic_callbacks_.insert_or_assign(name, callback);
}

void ComponentInterface::add_tf_broadcaster() {
  if (this->tf_broadcaster_ == nullptr) {
    RCLCPP_DEBUG(this->node_logging_->get_logger(), "Adding TF broadcaster.");
    console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_NONE);
    this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
        this->node_parameters_, this->node_topics_, tf2_ros::DynamicBroadcasterQoS());
  } else {
    RCLCPP_DEBUG(this->node_logging_->get_logger(), "TF broadcaster already exists.");
  }
}

void ComponentInterface::add_static_tf_broadcaster() {
  if (this->static_tf_broadcaster_ == nullptr) {
    RCLCPP_DEBUG(this->node_logging_->get_logger(), "Adding static TF broadcaster.");
    console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_NONE);
    tf2_ros::StaticBroadcasterQoS qos;
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
    this->static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->node_parameters_, this->node_topics_, qos, options);
  } else {
    RCLCPP_DEBUG(this->node_logging_->get_logger(), "Static TF broadcaster already exists.");
  }
}

void ComponentInterface::add_tf_listener() {
  if (this->tf_buffer_ == nullptr || this->tf_listener_ == nullptr) {
    RCLCPP_DEBUG(this->node_logging_->get_logger(), "Adding TF buffer and listener.");
    console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_NONE);
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->node_clock_->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
  } else {
    RCLCPP_DEBUG(this->node_logging_->get_logger(), "TF buffer and listener already exist.");
  }
}

void ComponentInterface::send_transform(const state_representation::CartesianPose& transform) {
  this->send_transforms(std::vector<state_representation::CartesianPose>{transform});
}

void ComponentInterface::send_transforms(const std::vector<state_representation::CartesianPose>& transforms) {
  this->publish_transforms(transforms, this->tf_broadcaster_);
}

void ComponentInterface::send_static_transform(const state_representation::CartesianPose& transform) {
  this->send_static_transforms(std::vector<state_representation::CartesianPose>{transform});
}

void ComponentInterface::send_static_transforms(const std::vector<state_representation::CartesianPose>& transforms) {
  this->publish_transforms(transforms, this->static_tf_broadcaster_, true);
}

state_representation::CartesianPose ComponentInterface::lookup_transform(
    const std::string& frame, const std::string& reference_frame, const tf2::TimePoint& time_point,
    const tf2::Duration& duration
) {
  auto transform = this->lookup_ros_transform(frame, reference_frame, time_point, duration);
  state_representation::CartesianPose result(frame, reference_frame);
  modulo_core::translators::read_message(result, transform);
  return result;
}

state_representation::CartesianPose ComponentInterface::lookup_transform(
    const std::string& frame, const std::string& reference_frame, double validity_period, const tf2::Duration& duration
) {
  auto transform =
      this->lookup_ros_transform(frame, reference_frame, tf2::TimePoint(std::chrono::microseconds(0)), duration);
  if (validity_period > 0.0
      && (this->node_clock_->get_clock()->now() - transform.header.stamp).seconds() > validity_period) {
    throw exceptions::LookupTransformException("Failed to lookup transform: Latest transform is too old!");
  }
  state_representation::CartesianPose result(frame, reference_frame);
  modulo_core::translators::read_message(result, transform);
  return result;
}

geometry_msgs::msg::TransformStamped ComponentInterface::lookup_ros_transform(
    const std::string& frame, const std::string& reference_frame, const tf2::TimePoint& time_point,
    const tf2::Duration& duration
) {
  if (this->tf_buffer_ == nullptr || this->tf_listener_ == nullptr) {
    throw exceptions::LookupTransformException("Failed to lookup transform: To TF buffer / listener configured.");
  }
  try {
    return this->tf_buffer_->lookupTransform(reference_frame, frame, time_point, duration);
  } catch (const tf2::TransformException& ex) {
    throw exceptions::LookupTransformException(std::string("Failed to lookup transform: ").append(ex.what()));
  }
}

rclcpp::QoS ComponentInterface::get_qos() const {
  return this->qos_;
}

void ComponentInterface::set_qos(const rclcpp::QoS& qos) {
  this->qos_ = qos;
}

void ComponentInterface::raise_error() {
  RCLCPP_DEBUG(this->node_logging_->get_logger(), "raise_error called: Setting predicate 'in_error_state' to true.");
  this->set_predicate("in_error_state", true);
}

void ComponentInterface::publish_predicate(const std::string& name) {
  modulo_component_interfaces::msg::Predicate message;
  message.component = this->node_base_->get_fully_qualified_name();
  message.predicate = name;
  message.value = this->get_predicate(name);
  this->predicate_publisher_->publish(message);
}

void ComponentInterface::publish_predicates() {
  for (const auto& predicate : this->predicates_) {
    this->publish_predicate(predicate.first);
  }
}

void ComponentInterface::publish_outputs() {
  for (const auto& [signal, publisher] : this->outputs_) {
    try {
      if (this->periodic_outputs_.at(signal)) {
        publisher->publish();
      }
    } catch (const modulo_core::exceptions::CoreException& ex) {
      RCLCPP_ERROR_STREAM_THROTTLE(this->node_logging_->get_logger(), *this->node_clock_->get_clock(), 1000,
                                   "Failed to publish output '" << signal << "': " << ex.what());
    }
  }
}

void ComponentInterface::evaluate_periodic_callbacks() {
  for (const auto& [name, callback] : this->periodic_callbacks_) {
    try {
      callback();
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_STREAM_THROTTLE(this->node_logging_->get_logger(), *this->node_clock_->get_clock(), 1000,
                                   "Failed to evaluate periodic function callback '" << name << "': " << ex.what());
    }
  }
}
}// namespace modulo_components
