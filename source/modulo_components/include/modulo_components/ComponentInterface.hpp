#pragma once

#include <rclcpp/node_interfaces/node_interfaces.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <state_representation/parameters/ParameterMap.hpp>

#include <modulo_core/Predicate.hpp>
#include <modulo_core/communication/PublisherHandler.hpp>
#include <modulo_core/communication/PublisherType.hpp>
#include <modulo_core/communication/SubscriptionHandler.hpp>
#include <modulo_core/exceptions.hpp>
#include <modulo_core/translators/parameter_translators.hpp>

#include <modulo_interfaces/msg/predicate_collection.hpp>
#include <modulo_interfaces/srv/empty_trigger.hpp>
#include <modulo_interfaces/srv/string_trigger.hpp>

#include <modulo_utils/parsing.hpp>

/**
 * @namespace modulo_components
 * @brief Modulo components
 */
namespace modulo_components {

/**
 * @struct ComponentServiceResponse
 * @brief Response structure to be returned by component services.
 * @details The structure contains a bool success field and a string message field.
 * This information is used to provide feedback on the service outcome to the service client.
 */
struct ComponentServiceResponse {
  bool success;
  std::string message;
};

/**
 * @class ComponentInterfacePublicInterface
 * @brief Friend class to the ComponentInterface to allow test fixtures to access protected and private members.
 */
class ComponentInterfacePublicInterface;

/**
 * @class ComponentInterface
 * @brief Base interface class for modulo components to wrap a ROS Node with custom behaviour.
 * @details This class is not intended for direct inheritance and usage by end-users. Instead, it defines the common
 * interfaces for the derived classes modulo_components::Component and modulo_components::LifecycleComponent.
 * @see Component, LifecycleComponent
 */
class ComponentInterface {
public:
  friend class ComponentInterfacePublicInterface;

  /**
   * @brief Virtual destructor
   */
  virtual ~ComponentInterface();

protected:
  /**
   * @brief Constructor with all node interfaces
   * @param interfaces Shared pointer to all the node interfaces of parent class
   */
  explicit ComponentInterface(
      const std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<ALL_RCLCPP_NODE_INTERFACES>>& interfaces);

  /**
   * @brief Get the component rate in Hertz
   * @return The component rate
  */
  double get_rate() const;

  /**
   * @brief Get the component period
   * @return The component period
  */
  template<typename T>
  T get_period() const;

  /**
   * @brief Step function that is called periodically.
   */
  virtual void step();

  /**
   * @brief Steps to execute periodically. To be redefined by derived Component classes.
   */
  virtual void on_step_callback();

  /**
   * @brief Add a parameter.
   * @details This method stores a pointer reference to an existing Parameter object in the local parameter map and
   * declares the equivalent ROS parameter on the ROS interface.
   * @param parameter A ParameterInterface pointer to a Parameter instance
   * @param description The description of the parameter
   * @param read_only If true, the value of the parameter cannot be changed after declaration
   * @throws ParameterException if the parameter could not be added
   */
  void add_parameter(
      const std::shared_ptr<state_representation::ParameterInterface>& parameter, const std::string& description,
      bool read_only = false);

  /**
   * @brief Add a parameter.
   * @details This method creates a new Parameter object instance to reference in the local parameter map and declares
   * the equivalent ROS parameter on the ROS interface.
   * @tparam T The type of the parameter
   * @param name The name of the parameter
   * @param value The value of the parameter
   * @param description The description of the parameter
   * @param read_only If true, the value of the parameter cannot be changed after declaration
   * @throws ParameterException if the parameter could not be added
   */
  template<typename T>
  void add_parameter(const std::string& name, const T& value, const std::string& description, bool read_only = false);

  /**
   * @brief Get a parameter by name.
   * @param name The name of the parameter
   * @throws modulo_core::exceptions::ParameterException if the parameter could not be found
   * @return The ParameterInterface pointer to a Parameter instance
   */
  [[nodiscard]] std::shared_ptr<state_representation::ParameterInterface> get_parameter(const std::string& name) const;

  /**
   * @brief Get a parameter value by name.
   * @tparam T The type of the parameter
   * @param name The name of the parameter
   * @throws modulo_core::exceptions::ParameterException if the parameter value could not be accessed
   * @return The value of the parameter
   */
  template<typename T>
  [[nodiscard]] T get_parameter_value(const std::string& name) const;

  /**
   * @brief Set the value of a parameter.
   * @details The parameter must have been previously declared. This method preserves the reference to the original
   * Parameter instance
   * @tparam T The type of the parameter
   * @param name The name of the parameter
   * @return The value of the parameter
   */
  template<typename T>
  void set_parameter_value(const std::string& name, const T& value);

  /**
   * @brief Parameter validation function to be redefined by derived Component classes.
   * @details This method is automatically invoked whenever the ROS interface tried to modify a parameter. Validation
   * and sanitization can be performed by reading or writing the value of the parameter through the ParameterInterface
   * pointer, depending on the parameter name and desired component behaviour. If the validation returns true, the
   * updated parameter value (including any modifications) is applied. If the validation returns false, any changes to
   * the parameter are discarded and the parameter value is not changed.
   * @param parameter A ParameterInterface pointer to a Parameter instance
   * @return The validation result
   */
  virtual bool
  on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>& parameter);

  /**
   * @brief Add a predicate to the map of predicates.
   * @param predicate_name the name of the associated predicate
   * @param predicate_value the boolean value of the predicate
   */
  void add_predicate(const std::string& predicate_name, bool predicate_value);

  /**
   * @brief Add a predicate to the map of predicates based on a function to periodically call.
   * @param predicate_name the name of the associated predicate
   * @param predicate_function the function to call that returns the value of the predicate
   */
  void add_predicate(const std::string& predicate_name, const std::function<bool(void)>& predicate_function);

  /**
   * @brief Get the logical value of a predicate.
   * @details If the predicate is not found or the callable function fails, the return value is false.
   * @param predicate_name the name of the predicate to retrieve from the map of predicates
   * @return the value of the predicate as a boolean
   */
  [[nodiscard]] bool get_predicate(const std::string& predicate_name) const;

  /**
   * @brief Set the value of the predicate given as parameter, if the predicate is not found does not do anything.
   * @details Even though the predicates are published periodically, the new value of this predicate will be published
   * once immediately after setting it.
   * @param predicate_name the name of the predicate to retrieve from the map of predicates
   * @param predicate_value the new value of the predicate
   */
  void set_predicate(const std::string& predicate_name, bool predicate_value);

  /**
   * @brief Set the value of the predicate given as parameter, if the predicate is not found does not do anything.
   * @details Even though the predicates are published periodically, the new value of this predicate will be published
   * once immediately after setting it.
   * @param predicate_name the name of the predicate to retrieve from the map of predicates
   * @param predicate_function the function to call that returns the value of the predicate
   */
  void set_predicate(const std::string& predicate_name, const std::function<bool(void)>& predicate_function);

  /**
   * @brief Add a trigger to the component. Triggers are predicates that are always false except when it's triggered in
   * which case it is set back to false immediately after it is read.
   * @param trigger_name The name of the trigger
   */
  void add_trigger(const std::string& trigger_name);

  /**
   * @brief Latch the trigger with the provided name.
   * @param trigger_name The name of the trigger
   */
  void trigger(const std::string& trigger_name);

  /**
   * @brief Declare an input to create the topic parameter without adding it to the map of inputs yet.
   * @param signal_name The signal name of the input
   * @param default_topic If set, the default value for the topic name to use
   * @param fixed_topic If true, the topic name of the signal is fixed
   * @throws modulo_core::exceptions::AddSignalException if the input could not be declared
   * (empty name or already created)
   */
  void declare_input(const std::string& signal_name, const std::string& default_topic = "", bool fixed_topic = false);

  /**
   * @brief Declare an output to create the topic parameter without adding it to the map of outputs yet.
   * @param signal_name The signal name of the output
   * @param default_topic If set, the default value for the topic name to use
   * @param fixed_topic If true, the topic name of the signal is fixed
   * @throws modulo_core::exceptions::AddSignalException if the output could not be declared
   * (empty name or already created)
   */
  void declare_output(const std::string& signal_name, const std::string& default_topic = "", bool fixed_topic = false);

  /**
   * @brief Add and configure an input signal of the component.
   * @tparam DataT Type of the data pointer
   * @param signal_name Name of the input signal
   * @param data Data to receive on the input signal
   * @param default_topic If set, the default value for the topic name to use
   * @param fixed_topic If true, the topic name of the input signal is fixed
   */
  template<typename DataT>
  void add_input(
      const std::string& signal_name, const std::shared_ptr<DataT>& data, const std::string& default_topic = "",
      bool fixed_topic = false);

  /**
   * @brief Add and configure an input signal of the component.
   * @tparam DataT Type of the data pointer
   * @param signal_name Name of the input signal
   * @param data Data to receive on the input signal
   * @param callback Callback function to trigger after receiving the input signal
   * @param default_topic If set, the default value for the topic name to use
   * @param fixed_topic If true, the topic name of the input signal is fixed
   */
  template<typename DataT>
  void add_input(
      const std::string& signal_name, const std::shared_ptr<DataT>& data, const std::function<void()>& callback,
      const std::string& default_topic = "", bool fixed_topic = false);

  /**
   * @brief Add and configure an input signal of the component.
   * @tparam MsgT The ROS message type of the subscription
   * @param signal_name Name of the input signal
   * @param callback The callback to use for the subscription
   * @param default_topic If set, the default value for the topic name to use
   * @param fixed_topic If true, the topic name of the input signal is fixed
   */
  template<typename MsgT>
  void add_input(
      const std::string& signal_name, const std::function<void(const std::shared_ptr<MsgT>)>& callback,
      const std::string& default_topic = "", bool fixed_topic = false);

  /**
   * @brief Helper function to parse the signal name and add an unconfigured PublisherInterface to the map of outputs.
   * @tparam DataT Type of the data pointer
   * @param signal_name Name of the output signal
   * @param data Data to transmit on the output signal
   * @param default_topic If set, the default value for the topic name to use
   * @param fixed_topic If true, the topic name of the output signal is fixed
   * @param publish_on_step If true, the output is published periodically on step
   * @throws modulo_core::exceptions::AddSignalException if the output could not be created
   * (empty name, already registered)
   * @return The parsed signal name
   */
  template<typename DataT>
  std::string create_output(
      modulo_core::communication::PublisherType publisher_type, const std::string& signal_name,
      const std::shared_ptr<DataT>& data, const std::string& default_topic, bool fixed_topic, bool publish_on_step);

  /**
   * @brief Trigger the publishing of an output
   * @param signal_name The name of the output signal
   * @throws ComponentException if the output is being published periodically or if the signal name could not be found
   */
  void publish_output(const std::string& signal_name);

  /**
   * @brief Remove an input from the map of inputs.
   * @param signal_name The name of the input
   */
  void remove_input(const std::string& signal_name);

  /**
   * @brief Remove an output from the map of outputs.
   * @param signal_name The name of the output
   */
  void remove_output(const std::string& signal_name);

  /**
   * @brief Add a service to trigger a callback function with no input arguments.
   * @param service_name The name of the service
   * @param callback A service callback function with no arguments that returns a ComponentServiceResponse
   */
  void add_service(const std::string& service_name, const std::function<ComponentServiceResponse(void)>& callback);

  /**
   * @brief Add a service to trigger a callback function with a string payload.
   * @details The string payload can have an arbitrary format to parameterize and control the callback behaviour
   * as desired. It is the responsibility of the service callback to parse the string according to some payload format.
   * When adding a service with a string payload, be sure to document the payload format appropriately.
   * @param service_name The name of the service
   * @param callback A service callback function with a string argument that returns a ComponentServiceResponse
   */
  void add_service(
      const std::string& service_name,
      const std::function<ComponentServiceResponse(const std::string& string)>& callback);

  /**
   * @brief Add a periodic callback function.
   * @details The provided function is evaluated periodically at the component step period.
   * @param name The name of the callback
   * @param callback The callback function that is evaluated periodically
   */
  void add_periodic_callback(const std::string& name, const std::function<void(void)>& callback);

  /**
   * @brief Configure a transform broadcaster.
   */
  void add_tf_broadcaster();

  /**
   * @brief Configure a static transform broadcaster.
   */
  void add_static_tf_broadcaster();

  /**
   * @brief Configure a transform buffer and listener.
   */
  void add_tf_listener();

  /**
   * @brief Send a transform to TF.
   * @param transform The transform to send
   */
  void send_transform(const state_representation::CartesianPose& transform);

  /**
   * @brief Send a vector of transforms to TF.
   * @param transforms The vector of transforms to send
   */
  void send_transforms(const std::vector<state_representation::CartesianPose>& transforms);

  /**
   * @brief Send a static transform to TF.
   * @param transform The transform to send
   */
  void send_static_transform(const state_representation::CartesianPose& transform);

  /**
   * @brief Send a vector of static transforms to TF.
   * @param transforms The vector of transforms to send
   */
  void send_static_transforms(const std::vector<state_representation::CartesianPose>& transforms);

  /**
   * @brief Look up a transform from TF.
   * @param frame The desired frame of the transform
   * @param reference_frame The desired reference frame of the transform
   * @param time_point The time at which the value of the transform is desired
   * @param duration How long to block the lookup call before failing
   * @throws modulo_core::exceptions::LookupTransformException if TF buffer/listener are unconfigured or
   * if the lookupTransform call failed
   * @return If it exists, the requested transform
   */
  [[nodiscard]] state_representation::CartesianPose lookup_transform(
      const std::string& frame, const std::string& reference_frame, const tf2::TimePoint& time_point,
      const tf2::Duration& duration);

  /**
   * @brief Look up a transform from TF.
   * @param frame The desired frame of the transform
   * @param reference_frame The desired reference frame of the transform
   * @param validity_period The validity period of the latest transform from the time of lookup in seconds
   * @param duration How long to block the lookup call before failing
   * @throws modulo_core::exceptions::LookupTransformException if TF buffer/listener are unconfigured,
   * if the lookupTransform call failed, or if the transform is too old
   * @return If it exists and is still valid, the requested transform
   */
  [[nodiscard]] state_representation::CartesianPose lookup_transform(
      const std::string& frame, const std::string& reference_frame = "world", double validity_period = -1.0,
      const tf2::Duration& duration = tf2::Duration(std::chrono::microseconds(10)));

  /**
   * @brief Getter of the Quality of Service attribute.
   * @return The Quality of Service attribute
   */
  [[nodiscard]] rclcpp::QoS get_qos() const;

  /**
   * @brief Set the Quality of Service for ROS publishers and subscribers.
   * @param qos The desired Quality of Service
   */
  void set_qos(const rclcpp::QoS& qos);

  /**
   * @brief Put the component in error state by setting the 'in_error_state' predicate to true.
   */
  virtual void raise_error();

  /**
   * @brief Helper function to publish all predicates.
   */
  void publish_predicates();

  /**
   * @brief Helper function to publish all output signals.
   */
  void publish_outputs();

  /**
   * @brief Helper function to evaluate all periodic function callbacks.
   */
  void evaluate_periodic_callbacks();

  std::map<std::string, std::shared_ptr<modulo_core::communication::SubscriptionInterface>> inputs_;///< Map of inputs
  std::map<std::string, std::shared_ptr<modulo_core::communication::PublisherInterface>> outputs_;  ///< Map of outputs
  std::map<std::string, bool> periodic_outputs_;///< Map of outputs with periodic publishing flag

private:
  /**
   * @brief Callback function to validate and update parameters on change.
   * @param parameters The new parameter objects provided by the ROS interface
   */
  void pre_set_parameters_callback(std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Callback function to notify ROS about the validation result from the pre_set_parameters_callback step.
   * @param parameters The new parameter objects provided by the ROS interface
   * @see pre_set_parameters_callback()
   * @return The result of the validation
   */
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Parameter validation function
   * @details This validates the period and calls the on_validate_parameter_callback function of the derived Component
   * classes.
   * @param parameter A ParameterInterface pointer to a Parameter instance
   * @return The validation result
   */
  bool validate_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter);

  /**
   * @brief Populate a Prediate message with the name and the value of a predicate.
   * @param name The name of the predicate
   * @param value The value of the predicate
  */
  modulo_interfaces::msg::Predicate get_predicate_message(const std::string& name, bool value) const;

  /**
   * @brief Declare a signal to create the topic parameter without adding it to the map of signals.
   * @param signal_name The name of the signal
   * @param type The type of the signal (input or output)
   * @param default_topic If set, the default value for the topic name to use
   * @param fixed_topic If true, the topic name of the signal is fixed
   * @throws modulo_core::exceptions::AddSignalException if the signal could not be declared
   * (empty name or already created)
   */
  void declare_signal(
      const std::string& signal_name, const std::string& type, const std::string& default_topic, bool fixed_topic);

  /**
   * @brief Remove a signal from the map of inputs or outputs.
   * @tparam T The type of the map entry (SubscriptionInterface or PublisherInterface)
   * @param signal_name The name of the signal to remove
   * @param signal_map The map of signals (either inputs or outputs)
   * @param skip_check If true, skip the check if the signal exists in the map and return false otherwise
   * @return True if the signal was removed, false if it didn't exist
   */
  template<typename T>
  bool remove_signal(
      const std::string& signal_name, std::map<std::string, std::shared_ptr<T>>& signal_map, bool skip_check = false);

  /**
   * @brief Validate an add_service request by parsing the service name and checking the maps of registered services.
   * @param service_name The name of the service
   * @param type One of empty|string
   * @throws modulo_core::exceptions::AddServiceException if the service could not be created
   * (empty name or already registered)
   * @return The parsed service name
   */
  std::string validate_service_name(const std::string& service_name, const std::string& type) const;

  /**
   * @brief Helper function to publish a predicate.
   * @param predicate_name The name of the predicate to publish
   * @param value The value of the predicate
   */
  void publish_predicate(const std::string& predicate_name, bool value) const;

  /**
   * @brief Helper function to send a vector of transforms through a transform broadcaster
   * @tparam T The type of the broadcaster (tf2_ros::TransformBroadcaster or tf2_ros::StaticTransformBroadcaster)
   * @param transforms The transforms to send
   * @param tf_broadcaster A pointer to a configured transform broadcaster object
   * @param is_static If true, treat the broadcaster as a static frame broadcaster for the sake of log messages
   */
  template<typename T>
  void publish_transforms(
      const std::vector<state_representation::CartesianPose>& transforms, const std::shared_ptr<T>& tf_broadcaster,
      bool is_static = false);

  /**
 * @brief Helper method to look up a transform from TF.
 * @param frame The desired frame of the transform
 * @param reference_frame The desired reference frame of the transform
 * @param time_point The time at which the value of the transform is desired
 * @param duration How long to block the lookup call before failing
 * @throws modulo_core::exceptions::LookupTransformException if TF buffer/listener are unconfigured or
 * if the lookupTransform call failed
 * @return If it exists, the requested transform
 */
  [[nodiscard]] geometry_msgs::msg::TransformStamped lookup_ros_transform(
      const std::string& frame, const std::string& reference_frame, const tf2::TimePoint& time_point,
      const tf2::Duration& duration);

  double rate_;          ///< The component rate in Hz
  double period_;        ///< The componet period in s
  std::mutex step_mutex_;///< Mutex for step callback

  std::map<std::string, modulo_core::Predicate> predicates_;///< Map of predicates
  std::shared_ptr<rclcpp::Publisher<modulo_interfaces::msg::PredicateCollection>>
      predicate_publisher_;///< Predicate publisher
  modulo_interfaces::msg::PredicateCollection predicate_message_;
  std::vector<std::string> triggers_;///< List of triggers

  std::map<std::string, std::shared_ptr<rclcpp::Service<modulo_interfaces::srv::EmptyTrigger>>>
      empty_services_;///< Map of EmptyTrigger services
  std::map<std::string, std::shared_ptr<rclcpp::Service<modulo_interfaces::srv::StringTrigger>>>
      string_services_;///< Map of StringTrigger services

  std::map<std::string, std::function<void(void)>> periodic_callbacks_;///< Map of periodic function callbacks

  state_representation::ParameterMap parameter_map_;///< ParameterMap for handling parameters
  std::unordered_map<std::string, bool> read_only_parameters_;
  std::shared_ptr<rclcpp::node_interfaces::PreSetParametersCallbackHandle>
      pre_set_parameter_cb_handle_;///< ROS callback function handle on pre set of parameters
  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle>
      on_set_parameter_cb_handle_;///< ROS callback function handle on set of parameters
  rcl_interfaces::msg::SetParametersResult set_parameters_result_;
  bool pre_set_parameter_callback_called_ = false;///< Flag to indicate if pre_set_parameter_callback was called

  std::shared_ptr<rclcpp::TimerBase> step_timer_;                             ///< Timer for the step function
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                ///< TF buffer
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                   ///< TF listener
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;             ///< TF broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;///< TF static broadcaster

  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_;
  std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_;
  std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_;
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters_;
  std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_;
  std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_;
  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_;
  rclcpp::QoS qos_ = rclcpp::QoS(10);///< Quality of Service for ROS publishers and subscribers
};

template<typename T>
inline void ComponentInterface::add_parameter(
    const std::string& name, const T& value, const std::string& description, bool read_only) {
  if (name.empty()) {
    RCLCPP_ERROR(this->node_logging_->get_logger(), "Failed to add parameter: Provide a non empty string as a name.");
    return;
  }
  this->add_parameter(state_representation::make_shared_parameter(name, value), description, read_only);
}

template<typename T>
inline T ComponentInterface::get_parameter_value(const std::string& name) const {
  try {
    return this->parameter_map_.template get_parameter_value<T>(name);
  } catch (const state_representation::exceptions::InvalidParameterException& ex) {
    throw modulo_core::exceptions::ParameterException(
        "Failed to get parameter value of parameter '" + name + "': " + ex.what());
  }
}

template<typename T>
inline void ComponentInterface::set_parameter_value(const std::string& name, const T& value) {
  try {
    std::vector<rclcpp::Parameter> parameters{
        modulo_core::translators::write_parameter(state_representation::make_shared_parameter(name, value))};
    this->pre_set_parameters_callback(parameters);
    this->pre_set_parameter_callback_called_ = true;
    auto result = this->node_parameters_->set_parameters(parameters).at(0);
    if (!result.successful) {
      RCLCPP_ERROR_STREAM_THROTTLE(
          this->node_logging_->get_logger(), *this->node_clock_->get_clock(), 1000,
          "Failed to set parameter value of parameter '" << name << "': " << result.reason);
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM_THROTTLE(
        this->node_logging_->get_logger(), *this->node_clock_->get_clock(), 1000,
        "Failed to set parameter value of parameter '" << name << "': " << ex.what());
  }
}

template<typename DataT>
inline void ComponentInterface::add_input(
    const std::string& signal_name, const std::shared_ptr<DataT>& data, const std::string& default_topic,
    bool fixed_topic) {
  this->add_input(signal_name, data, [] {}, default_topic, fixed_topic);
}

template<typename DataT>
inline void ComponentInterface::add_input(
    const std::string& signal_name, const std::shared_ptr<DataT>& data, const std::function<void()>& user_callback,
    const std::string& default_topic, bool fixed_topic) {
  using namespace modulo_core::communication;
  try {
    if (data == nullptr) {
      throw modulo_core::exceptions::NullPointerException("Invalid data pointer for input '" + signal_name + "'.");
    }
    this->declare_input(signal_name, default_topic, fixed_topic);
    std::string parsed_signal_name = modulo_utils::parsing::parse_topic_name(signal_name);
    auto topic_name = this->get_parameter_value<std::string>(parsed_signal_name + "_topic");
    RCLCPP_DEBUG_STREAM(
        this->node_logging_->get_logger(),
        "Adding input '" << parsed_signal_name << "' with topic name '" << topic_name << "'.");
    auto message_pair = make_shared_message_pair(data, this->node_clock_->get_clock());
    std::shared_ptr<SubscriptionInterface> subscription_interface;
    switch (message_pair->get_type()) {
      case MessageType::BOOL: {
        auto subscription_handler = std::make_shared<SubscriptionHandler<std_msgs::msg::Bool>>(message_pair);
        auto subscription = rclcpp::create_subscription<std_msgs::msg::Bool>(
            this->node_parameters_, this->node_topics_, topic_name, this->qos_,
            subscription_handler->get_callback(user_callback));
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
      case MessageType::FLOAT64: {
        auto subscription_handler = std::make_shared<SubscriptionHandler<std_msgs::msg::Float64>>(message_pair);
        auto subscription = rclcpp::create_subscription<std_msgs::msg::Float64>(
            this->node_parameters_, this->node_topics_, topic_name, this->qos_,
            subscription_handler->get_callback(user_callback));
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
      case MessageType::FLOAT64_MULTI_ARRAY: {
        auto subscription_handler =
            std::make_shared<SubscriptionHandler<std_msgs::msg::Float64MultiArray>>(message_pair);
        auto subscription = rclcpp::create_subscription<std_msgs::msg::Float64MultiArray>(
            this->node_parameters_, this->node_topics_, topic_name, this->qos_,
            subscription_handler->get_callback(user_callback));
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
      case MessageType::INT32: {
        auto subscription_handler = std::make_shared<SubscriptionHandler<std_msgs::msg::Int32>>(message_pair);
        auto subscription = rclcpp::create_subscription<std_msgs::msg::Int32>(
            this->node_parameters_, this->node_topics_, topic_name, this->qos_,
            subscription_handler->get_callback(user_callback));
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
      case MessageType::STRING: {
        auto subscription_handler = std::make_shared<SubscriptionHandler<std_msgs::msg::String>>(message_pair);
        auto subscription = rclcpp::create_subscription<std_msgs::msg::String>(
            this->node_parameters_, this->node_topics_, topic_name, this->qos_,
            subscription_handler->get_callback(user_callback));
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
      case MessageType::ENCODED_STATE: {
        auto subscription_handler = std::make_shared<SubscriptionHandler<modulo_core::EncodedState>>(message_pair);
        auto subscription = rclcpp::create_subscription<modulo_core::EncodedState>(
            this->node_parameters_, this->node_topics_, topic_name, this->qos_,
            subscription_handler->get_callback(user_callback));
        subscription_interface = subscription_handler->create_subscription_interface(subscription);
        break;
      }
    }
    this->inputs_.insert_or_assign(parsed_signal_name, subscription_interface);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(
        this->node_logging_->get_logger(), "Failed to add input '" << signal_name << "': " << ex.what());
  }
}

template<typename MsgT>
inline void ComponentInterface::add_input(
    const std::string& signal_name, const std::function<void(const std::shared_ptr<MsgT>)>& callback,
    const std::string& default_topic, bool fixed_topic) {
  using namespace modulo_core::communication;
  try {
    std::string parsed_signal_name = modulo_utils::parsing::parse_topic_name(signal_name);
    this->declare_input(parsed_signal_name, default_topic, fixed_topic);
    auto topic_name = this->get_parameter_value<std::string>(parsed_signal_name + "_topic");
    RCLCPP_DEBUG_STREAM(
        this->node_logging_->get_logger(),
        "Adding input '" << parsed_signal_name << "' with topic name '" << topic_name << "'.");
    auto subscription =
        rclcpp::create_subscription<MsgT>(this->node_parameters_, this->node_topics_, topic_name, this->qos_, callback);
    auto subscription_interface =
        std::make_shared<SubscriptionHandler<MsgT>>()->create_subscription_interface(subscription);
    this->inputs_.insert_or_assign(parsed_signal_name, subscription_interface);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(
        this->node_logging_->get_logger(), "Failed to add input '" << signal_name << "': " << ex.what());
  }
}

template<typename DataT>
inline std::string ComponentInterface::create_output(
    modulo_core::communication::PublisherType publisher_type, const std::string& signal_name,
    const std::shared_ptr<DataT>& data, const std::string& default_topic, bool fixed_topic, bool publish_on_step) {
  using namespace modulo_core::communication;
  try {
    if (data == nullptr) {
      throw modulo_core::exceptions::NullPointerException("Invalid data pointer for output '" + signal_name + "'.");
    }
    this->declare_output(signal_name, default_topic, fixed_topic);
    auto parsed_signal_name = modulo_utils::parsing::parse_topic_name(signal_name);
    RCLCPP_DEBUG_STREAM(
        this->node_logging_->get_logger(),
        "Creating output '" << parsed_signal_name << "' (provided signal name was '" << signal_name << "').");
    auto message_pair = make_shared_message_pair(data, this->node_clock_->get_clock());
    this->outputs_.insert_or_assign(
        parsed_signal_name, std::make_shared<PublisherInterface>(publisher_type, message_pair));
    this->periodic_outputs_.insert_or_assign(parsed_signal_name, publish_on_step);
    return parsed_signal_name;
  } catch (const modulo_core::exceptions::AddSignalException&) {
    throw;
  } catch (const std::exception& ex) {
    throw modulo_core::exceptions::AddSignalException(ex.what());
  }
}

template<typename T>
inline bool ComponentInterface::remove_signal(
    const std::string& signal_name, std::map<std::string, std::shared_ptr<T>>& signal_map, bool skip_check) {
  if (!skip_check && signal_map.find(signal_name) == signal_map.cend()) {
    return false;
  } else {
    RCLCPP_DEBUG_STREAM(this->node_logging_->get_logger(), "Removing signal '" << signal_name << "'.");
    signal_map.at(signal_name).reset();
    return signal_map.erase(signal_name);
  }
}

template<typename T>
inline void ComponentInterface::publish_transforms(
    const std::vector<state_representation::CartesianPose>& transforms, const std::shared_ptr<T>& tf_broadcaster,
    bool is_static) {
  std::string modifier = is_static ? "static " : "";
  if (tf_broadcaster == nullptr) {
    RCLCPP_ERROR_STREAM_THROTTLE(
        this->node_logging_->get_logger(), *this->node_clock_->get_clock(), 1000,
        "Failed to send " << modifier << "transform: No " << modifier << "TF broadcaster configured.");
    return;
  }
  try {
    std::vector<geometry_msgs::msg::TransformStamped> transform_messages;
    transform_messages.reserve(transforms.size());
    for (const auto& tf : transforms) {
      geometry_msgs::msg::TransformStamped transform_message;
      modulo_core::translators::write_message(transform_message, tf, this->node_clock_->get_clock()->now());
      transform_messages.emplace_back(transform_message);
    }
    tf_broadcaster->sendTransform(transform_messages);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM_THROTTLE(
        this->node_logging_->get_logger(), *this->node_clock_->get_clock(), 1000,
        "Failed to send " << modifier << "transform: " << ex.what());
  }
}
}// namespace modulo_components
