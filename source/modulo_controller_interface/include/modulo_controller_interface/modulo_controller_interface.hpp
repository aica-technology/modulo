#pragma once

#include <mutex>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/helpers.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <state_representation/parameters/ParameterMap.hpp>

#include <modulo_core/EncodedState.hpp>
#include <modulo_core/communication/MessagePair.hpp>
#include <modulo_core/translators/message_writers.hpp>
#include <modulo_core/translators/parameter_translators.hpp>
#include <modulo_interfaces/msg/predicate_collection.hpp>

#include "modulo_controller_interface/utils/utilities.hpp"

#include <modulo_component_interfaces/srv/empty_trigger.hpp>
#include <modulo_component_interfaces/srv/string_trigger.hpp>

namespace modulo_controller_interface {

typedef std::variant<
    std::shared_ptr<rclcpp::Subscription<modulo_core::EncodedState>>,
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>>,
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>>,
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>>,
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>>,
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>>
    SubscriptionVariant;

typedef std::variant<
    realtime_tools::RealtimeBuffer<std::shared_ptr<modulo_core::EncodedState>>,
    realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Bool>>,
    realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>>,
    realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>>,
    realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Int32>>,
    realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::String>>>
    BufferVariant;

typedef std::tuple<
    std::shared_ptr<state_representation::State>,
    std::shared_ptr<rclcpp::Publisher<modulo_core::EncodedState>>,
    realtime_tools::RealtimePublisherSharedPtr<modulo_core::EncodedState>>
    EncodedStatePublishers;
typedef std::pair<
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>,
    realtime_tools::RealtimePublisherSharedPtr<std_msgs::msg::Bool>>
    BoolPublishers;
typedef std::pair<
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>>,
    realtime_tools::RealtimePublisherSharedPtr<std_msgs::msg::Float64>>
    DoublePublishers;
typedef std::pair<
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>,
    realtime_tools::RealtimePublisherSharedPtr<std_msgs::msg::Float64MultiArray>>
    DoubleVecPublishers;
typedef std::pair<
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>>,
    realtime_tools::RealtimePublisherSharedPtr<std_msgs::msg::Int32>>
    IntPublishers;
typedef std::pair<
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>>,
    realtime_tools::RealtimePublisherSharedPtr<std_msgs::msg::String>>
    StringPublishers;

typedef std::variant<
    EncodedStatePublishers, BoolPublishers, DoublePublishers, DoubleVecPublishers, IntPublishers, StringPublishers>
    PublisherVariant;

/**
 * @struct ControllerInput
 * @brief Input structure to save topic data in a realtime buffer and timestamps in one object.
*/
struct ControllerInput {
  ControllerInput(BufferVariant buffer_variant) : buffer(std::move(buffer_variant)) {}
  BufferVariant buffer;
  std::chrono::time_point<std::chrono::steady_clock> timestamp;
};

/**
 * @struct ControllerServiceResponse
 * @brief Response structure to be returned by controller services.
 * @details The structure contains a bool success field and a string message field.
 * This information is used to provide feedback on the service outcome to the service client.
 */
struct ControllerServiceResponse {
  bool success;
  std::string message;
};

/**
 * @class ModuloControllerInterface
 * @brief Base controller class to combine ros2_control, control libraries and modulo.
 */
class ModuloControllerInterface : public controller_interface::ControllerInterface {
public:
  /**
   * @brief Default constructor
   * @param claim_all_state_interfaces Flag to indicate if all state interfaces should be claimed
   */
  ModuloControllerInterface(bool claim_all_state_interfaces = false);

  /**
   * @brief Declare parameters and register the on_set_parameters callback.
   * @return CallbackReturn status
   */
  CallbackReturn on_init() override;

  /**
   * @brief Set class properties from parameters and add signals.
   * @details This functions calls the internal on_configure() method
   * @param previous_state The previous lifecycle state
   * @return SUCCESS or ERROR
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;

  /**
   * @brief Initialize internal data attributes from configured interfaces and wait for valid states from hardware.
   * @details This functions calls the internal on_activate() method
   * @param previous_state The previous lifecycle state
   * @return SUCCESS or ERROR
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;

  /**
   * @brief Deactivate the controller.
   * @param previous_state The previous lifecycle state
   * @return SUCCESS or ERROR
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

  /**
   * @brief Read the state interfaces, perform control evaluation and write the command interfaces.
   * @param time The controller clock time
   * @param period Time elapsed since the last control evaluation
   * @return OK or ERROR
   */
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) final;

  /**
   * @brief Configure the state interfaces.
   * @return The state interface configuration
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const final;

  /**
   * @brief Configure the command interfaces.
   * @return The command interface configuration
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const final;

protected:
  /**
   * @brief Configure the controller.
   * @details This method should be overridden by derived classes.
   * @return SUCCESS or ERROR
   */
  virtual CallbackReturn on_configure();

  /**
   * @brief Activate the controller.
   * @details This method should be overridden by derived classes.
   * @return SUCCESS or ERROR
   */
  virtual CallbackReturn on_activate();

  /**
   * @brief Deactivate the controller.
   * @details This method should be overridden by derived classes.
   * @return SUCCESS or ERROR
   */
  virtual CallbackReturn on_deactivate();

  /**
   * @brief Read the state interfaces.
   * @return OK or ERROR
   */
  virtual controller_interface::return_type read_state_interfaces();

  /**
   * @brief Write the command interfaces.
   * @param period Time elapsed since the last control evaluation
   * @return OK or ERROR
   */
  virtual controller_interface::return_type write_command_interfaces(const rclcpp::Duration& period);

  /**
   * @brief The control logic callback.
   * @details This method should be overridden by derived classes.
   * It is called in the update() method between reading the state interfaces and writing the command interfaces.
   * @param time The controller clock time
   * @param period Time elapsed since the last control evaluation
   * @return OK or ERROR
   */
  virtual controller_interface::return_type
  evaluate(const rclcpp::Time& time, const std::chrono::nanoseconds& period) = 0;

  /**
   * @brief Add a state interface to the controller by name.
   * @param name The name of the parent tag, e.g. the name of the joint, sensor or gpio
   * @param interface The desired state interface
  */
  void add_state_interface(const std::string& name, const std::string& interface);

  /**
   * @brief Add a command interface to the controller by name.
   * @param name The name of the parent tag, e.g. the name of the joint, sensor or gpio
   * @param interface The desired command interface
  */
  void add_command_interface(const std::string& name, const std::string& interface);

  /**
   * @brief Get a map containing the state interfaces by name of the parent tag.
   * @param name The name of the parent tag, e.g. the name of the joint, sensor or gpio
   * @throw out_of_range if the desired state interface is not available
   * @return The map containing the values of the state interfaces
  */
  std::unordered_map<std::string, double> get_state_interfaces(const std::string& name) const;

  /**
   * @brief Get the value of a state interface by name.
   * @param name The name of the parent tag, e.g. the name of the joint, sensor or gpio
   * @param interface The desired state interface
   * @throw out_of_range if the desired state interface is not available
   * @return The value of the state interface
  */
  double get_state_interface(const std::string& name, const std::string& interface) const;

  /**
   * @brief Get the value of a command interface by name.
   * @param name The name of the parent tag, e.g. the name of the joint, sensor or gpio
   * @param interface The desired command interface
   * @throw out_of_range if the desired command interface is not available
   * @return The value of the command interface
  */
  double get_command_interface(const std::string& name, const std::string& interface) const;

  /**
   * @brief Set the value of a command interface by name.
   * @param name The name of the parent tag, e.g. the name of the sensor or gpio
   * @param interface The name of the command interface
   * @param value The new value of the interface
  */
  void set_command_interface(const std::string& name, const std::string& interface, double value);

  /**
   * @brief Add a parameter.
   * @details This method stores a pointer reference to an existing Parameter object in the local parameter map and
   * declares the equivalent ROS parameter on the ROS interface.
   * @param parameter A ParameterInterface pointer to a Parameter instance
   * @param description The description of the parameter
   * @param read_only If true, the value of the parameter cannot be changed after declaration
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
   */
  template<typename T>
  void add_parameter(const std::string& name, const T& value, const std::string& description, bool read_only = false);

  /**
   * @brief Parameter validation function to be redefined by derived controller classes.
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
   * @brief Get a parameter by name.
   * @param name The name of the parameter
   * @return The ParameterInterface pointer to a Parameter instance
   */
  [[nodiscard]] std::shared_ptr<state_representation::ParameterInterface> get_parameter(const std::string& name) const;

  /**
   * @brief Get a parameter value by name.
   * @tparam T The type of the parameter
   * @param name The name of the parameter
   * @return The value of the parameter
   */
  template<typename T>
  T get_parameter_value(const std::string& name) const;

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
   * @brief Add an input to the controller
   * @details Inputs should be added in the on_init function of the derived controllers. Doing this will create a ROS 2
   * subscription for the message type.
   * @tparam T The type of the input data
   * @param name The name of the input
   * @param topic_name The topic name of the input (defaults to ~/name)
   */
  template<typename T>
  void add_input(const std::string& name, const std::string& topic_name = "");

  /**
   * @brief Add an output to the controller
   * @details Outputs should be added in the on_init function of the derived controllers. Doing this will create a ROS 2
   * publisher for the message type and wrap it in a RealtimePublisher.
   * @tparam T The type of the output data
   * @param name The name of the output
   * @param topic_name  The topic name of the output (defaults to ~/name)
   */
  template<typename T>
  void add_output(const std::string& name, const std::string& topic_name = "");

  /**
   * @brief Read the most recent message of an input
   * @tparam T The expected type of the input data
   * @param name The name of the input
   * @return The data on the desired input channel if the input exists and is not older than parameter
   * 'input_validity_period'
   */
  template<typename T>
  std::optional<T> read_input(const std::string& name);

  /**
   * @brief Write an object to an output
   * @details This uses the realtime publisher from add_output() to simplify publishing data in the realtime context of
   * the control loop.
   * @tparam T The type of the the object to publish
   * @param name The name of the output
   * @param state The object to publish
   */
  template<typename T>
  void write_output(const std::string& name, const T& data);

  /**
   * @brief Add a service to trigger a callback function with no input arguments.
   * @param service_name The name of the service
   * @param callback A service callback function with no arguments that returns a ControllerServiceResponse
   */
  void add_service(const std::string& service_name, const std::function<ControllerServiceResponse(void)>& callback);

  /**
   * @brief Add a service to trigger a callback function with a string payload.
   * @details The string payload can have an arbitrary format to parameterize and control the callback behaviour
   * as desired. It is the responsibility of the service callback to parse the string according to some payload format.
   * When adding a service with a string payload, be sure to document the payload format appropriately.
   * @param service_name The name of the service
   * @param callback A service callback function with a string argument that returns a ControllerServiceResponse
   */
  void add_service(
      const std::string& service_name,
      const std::function<ControllerServiceResponse(const std::string& string)>& callback);

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
   * @brief Check if the controller is currently in state active or not.
   * @return True if the controller is active, false otherwise
   */
  bool is_active() const;

  std::string hardware_name_;///< The hardware name provided by a parameter

private:
  /**
   * @brief Helper to add an interface to the list of desired interfaces (state or command)
   * @param name The name of the parent tag, e.g. the name of the joint, sensor or gpio
   * @param interface The desired state interface
   * @param list The list to add to passed as reference
   * @param type The type of interface (state or command)
  */
  void add_interface(
      const std::string& name, const std::string& interface, std::vector<std::string>& list, const std::string& type);

  /**
   * @brief Add a predicate to the map of predicates.
   * @param name The name of the predicate
   * @param predicate The predicate variant
   */
  void add_variant_predicate(const std::string& name, const utilities::PredicateVariant& predicate);

  /**
   * @brief Get the logical value of a predicate.
   * @details If the predicate is not found or the callable function fails, the return value is false.
   * @param predicate_name the name of the predicate to retrieve from the map of predicates
   * @return the value of the predicate as a boolean
   */
  [[nodiscard]] bool get_predicate(const std::string& predicate_name) const;

  /**
   * @brief Set the predicate given as parameter, if the predicate is not found does not do anything.
   * @param name The name of the predicate
   * @param predicate The predicate variant
   */
  void set_variant_predicate(const std::string& name, const utilities::PredicateVariant& predicate);

  /**
   * @brief Populate a Prediate message with the name and the value of a predicate.
   * @param name The name of the predicate
  */
  modulo_interfaces::msg::Predicate get_predicate_message(const std::string& name) const;

  /**
   * @brief Helper function to publish a predicate.
   * @param name The name of the predicate to publish
   */
  void publish_predicate(const std::string& name) const;

  /**
   * @brief Helper function to publish all predicates.
   */
  void publish_predicates() const;

  /**
   * @brief Create an input from a message pair, name, and topic name
   * @details This function checks if the input already exists, sanitizes the name and topic name, and adds the
   * parameter for the topic name
   * @param input The controller input
   * @param name The name of the input
   * @param topic_name The topic name of the input
  */
  void create_input(const ControllerInput& input, const std::string& name, const std::string& topic_name);

  /**
   * @brief Create an output from a publisher variant, name, and topic name
   * @details This function checks if the output already exists, sanitizes the name and topic name, and adds the
   * parameter for the topic name
   * @param publishers The output publisher variant
   * @param name The name of the output
   * @param topic_name The topic name of the output
  */
  void create_output(const PublisherVariant& publishers, const std::string& name, const std::string& topic_name);

  /**
   * @brief Create a subscription for an input
   * @tparam T The message type of the subscription
   * @param name The name of the input
   * @param topic_name The topic name of the input
  */
  template<typename T>
  std::shared_ptr<rclcpp::Subscription<T>> create_subscription(const std::string& name, const std::string& topic_name);

  /**
   * @brief This function is called on read_input and checks if the data on that input channel is valid
   * @param name The name of the input
  */
  bool check_input_valid(const std::string& name) const;

  /**
   * @brief Helper function to publish an output of type std_msgs::msg::XXX.
  */
  template<typename PublisherT, typename MsgT, typename T>
  void write_std_output(const std::string& name, const T& data);

  /**
   * @brief Go through the declared inputs and create subscriptions for each one of them.
   */
  void add_inputs();

  /**
   * @brief Go through the declared outputs and create publishers for each one of them.
   */
  void add_outputs();

  /**
   * @brief Validate an add_service request by parsing the service name and checking the maps of registered services.
   * @param service_name The name of the service
   * @param type One of 'empty' or 'string'
   * @return The parsed service name
   */
  std::string validate_service_name(const std::string& service_name, const std::string& type) const;

  /**
   * @brief Callback function to validate and update parameters on change.
   * @param parameters The new parameter objects provided by the ROS interface
   * @return The result of the validation
   */
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter>& parameters);

  using controller_interface::ControllerInterfaceBase::command_interfaces_;
  using controller_interface::ControllerInterfaceBase::state_interfaces_;

  state_representation::ParameterMap parameter_map_;///< ParameterMap for handling parameters
  bool set_parameter_callback_called_ = false;      ///< Flag to indicate if on_set_parameter_callback was called
  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle>
      parameter_cb_handle_;///< ROS callback function handle for setting parameters

  std::vector<SubscriptionVariant> subscriptions_;///< Vector of subscriptions
  std::map<std::string, ControllerInput> inputs_;///< Map of inputs
  std::map<std::string, std::shared_ptr<modulo_core::communication::MessagePairInterface>>
      input_message_pairs_;///< Map of message pairs for EncodedState inputs
  std::map<std::string, PublisherVariant> outputs_;///< Map of outputs
  double input_validity_period_;///< Maximum age of an input state before discarding it as expired
  rclcpp::QoS qos_ = rclcpp::QoS(10); ///< Quality of Service for ROS publishers, subscribers and services

  std::map<std::string, std::shared_ptr<rclcpp::Service<modulo_component_interfaces::srv::EmptyTrigger>>>
      empty_services_;///< Map of EmptyTrigger services
  std::map<std::string, std::shared_ptr<rclcpp::Service<modulo_component_interfaces::srv::StringTrigger>>>
      string_services_;///< Map of StringTrigger services

  std::map<std::string, utilities::PredicateVariant> predicates_; ///< Map of predicates
  std::shared_ptr<rclcpp::Publisher<modulo_interfaces::msg::PredicateCollection>>
      predicate_publisher_; ///< Predicate publisher
  std::map<std::string, bool> triggers_; ///< Map of triggers
  modulo_interfaces::msg::PredicateCollection predicate_message_;
  std::shared_ptr<rclcpp::TimerBase> predicate_timer_;

  std::unordered_map<std::string, std::unordered_map<std::string, double>>
      state_interface_data_;///< Map of all state interface data
  std::vector<double> command_interface_data_;///< Vector of all command interface data
  std::unordered_map<std::string, std::unordered_map<std::string, unsigned int>>
      command_interface_indices_;                   ///< Map of command interface indices in the command interface data
  std::vector<std::string> state_interface_names_;  ///< List of state interfaces to claim
  std::vector<std::string> command_interface_names_;///< List of command interfaces to claim
  bool claim_all_state_interfaces_;                 ///< Flag to decide if all state interfaces are claimed

  std::timed_mutex command_mutex_;
  // TODO make missed_locks an internal parameter
  unsigned int missed_locks_;
};

template<typename T>
inline void ModuloControllerInterface::add_parameter(
    const std::string& name, const T& value, const std::string& description, bool read_only) {
  if (name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to add parameter: Provide a non empty string as a name.");
    return;
  }
  add_parameter(state_representation::make_shared_parameter(name, value), description, read_only);
}

inline void ModuloControllerInterface::add_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>& parameter, const std::string& description,
    bool read_only) {
  set_parameter_callback_called_ = false;
  rclcpp::Parameter ros_param;
  try {
    ros_param = modulo_core::translators::write_parameter(parameter);
    if (!get_node()->has_parameter(parameter->get_name())) {
      RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Adding parameter '" << parameter->get_name() << "'.");
      parameter_map_.set_parameter(parameter);
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description = description;
      descriptor.read_only = read_only;
      if (parameter->is_empty()) {
        descriptor.dynamic_typing = true;
        descriptor.type = modulo_core::translators::get_ros_parameter_type(parameter->get_parameter_type());
        get_node()->declare_parameter(parameter->get_name(), rclcpp::ParameterValue{}, descriptor);
      } else {
        get_node()->declare_parameter(parameter->get_name(), ros_param.get_parameter_value(), descriptor);
      }
      if (!set_parameter_callback_called_) {
        auto result = on_set_parameters_callback({get_node()->get_parameters({parameter->get_name()})});
        if (!result.successful) {
          get_node()->undeclare_parameter(parameter->get_name());
          throw std::runtime_error(result.reason);
        }
      }
    } else {
      RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Parameter '" << parameter->get_name() << "' already exists.");
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(
        get_node()->get_logger(), "Failed to add parameter '" << parameter->get_name() << "': " << ex.what());
  }
}

inline std::shared_ptr<state_representation::ParameterInterface>
ModuloControllerInterface::get_parameter(const std::string& name) const {
  return parameter_map_.get_parameter(name);
}

template<typename T>
inline T ModuloControllerInterface::get_parameter_value(const std::string& name) const {
  return parameter_map_.template get_parameter_value<T>(name);
}

template<typename T>
inline void ModuloControllerInterface::add_input(const std::string& name, const std::string& topic_name) {
  auto buffer = realtime_tools::RealtimeBuffer<std::shared_ptr<modulo_core::EncodedState>>();
  auto input = ControllerInput(buffer);
  create_input(input, name, topic_name);
  input_message_pairs_.insert_or_assign(
      name, modulo_core::communication::make_shared_message_pair(std::make_shared<T>(), get_node()->get_clock()));
}

template<>
inline void ModuloControllerInterface::add_input<bool>(const std::string& name, const std::string& topic_name) {
  auto buffer = realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Bool>>();
  auto input = ControllerInput(buffer);
  create_input(input, name, topic_name);
}

template<>
inline void ModuloControllerInterface::add_input<double>(const std::string& name, const std::string& topic_name) {
  auto buffer = realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>>();
  auto input = ControllerInput(buffer);
  create_input(input, name, topic_name);
}

template<>
inline void
ModuloControllerInterface::add_input<std::vector<double>>(const std::string& name, const std::string& topic_name) {
  auto buffer = realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>>();
  auto input = ControllerInput(buffer);
  create_input(input, name, topic_name);
}

template<>
inline void ModuloControllerInterface::add_input<int>(const std::string& name, const std::string& topic_name) {
  auto buffer = realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Int32>>();
  auto input = ControllerInput(buffer);
  create_input(input, name, topic_name);
}

template<>
inline void ModuloControllerInterface::add_input<std::string>(const std::string& name, const std::string& topic_name) {
  auto buffer = realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::String>>();
  auto input = ControllerInput(buffer);
  create_input(input, name, topic_name);
}

template<typename T>
inline std::shared_ptr<rclcpp::Subscription<T>>
ModuloControllerInterface::create_subscription(const std::string& name, const std::string& topic_name) {
  return get_node()->create_subscription<T>(topic_name, this->qos_, [this, name](const std::shared_ptr<T> message) {
    std::get<realtime_tools::RealtimeBuffer<std::shared_ptr<T>>>(this->inputs_.at(name).buffer).writeFromNonRT(message);
    this->inputs_.at(name).timestamp = std::chrono::steady_clock::now();
  });
}

template<typename T>
inline void ModuloControllerInterface::add_output(const std::string& name, const std::string& topic_name) {
  std::shared_ptr<state_representation::State> state_ptr = std::make_shared<T>();
  create_output(EncodedStatePublishers(state_ptr, {}, {}), name, topic_name);
}

template<>
inline void ModuloControllerInterface::add_output<bool>(const std::string& name, const std::string& topic_name) {
  create_output(BoolPublishers(), name, topic_name);
}

template<>
inline void ModuloControllerInterface::add_output<double>(const std::string& name, const std::string& topic_name) {
  create_output(DoublePublishers(), name, topic_name);
}

template<>
inline void
ModuloControllerInterface::add_output<std::vector<double>>(const std::string& name, const std::string& topic_name) {
  create_output(DoubleVecPublishers(), name, topic_name);
}

template<>
inline void ModuloControllerInterface::add_output<int>(const std::string& name, const std::string& topic_name) {
  create_output(IntPublishers(), name, topic_name);
}

template<>
inline void ModuloControllerInterface::add_output<std::string>(const std::string& name, const std::string& topic_name) {
  create_output(StringPublishers(), name, topic_name);
}

template<typename T>
inline std::optional<T> ModuloControllerInterface::read_input(const std::string& name) {
  if (!check_input_valid(name)) {
    return {};
  }
  auto message = *std::get<realtime_tools::RealtimeBuffer<std::shared_ptr<modulo_core::EncodedState>>>(
                      this->inputs_.at(name).buffer)
                      .readFromNonRT();
  std::shared_ptr<state_representation::State> state;
  try {
    auto message_pair = input_message_pairs_.at(name);
    message_pair->read<modulo_core::EncodedState, state_representation::State>(*message);
    state = message_pair->get_message_pair<modulo_core::EncodedState, state_representation::State>()->get_data();
  } catch (const std::exception& ex) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Could not read EncodedState message on input '%s': %s", name.c_str(), ex.what());
    return {};
  }
  if (state->is_empty()) {
    return {};
  }
  try {
    return *std::dynamic_pointer_cast<T>(state);
  } catch (const std::exception& ex) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000, "Could not cast input '%s' to desired state type: %s",
        name.c_str(), ex.what());
  }
  return {};
}

template<>
inline std::optional<bool> ModuloControllerInterface::read_input<bool>(const std::string& name) {
  if (!check_input_valid(name)) {
    return {};
  }
  // no need to check for emptiness of the pointer: timestamps are default constructed to 0, so an input being valid
  //  means that a message was received
  return (*std::get<realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Bool>>>(this->inputs_.at(name).buffer)
               .readFromNonRT())
      ->data;
}

template<>
inline std::optional<double> ModuloControllerInterface::read_input<double>(const std::string& name) {
  if (!check_input_valid(name)) {
    return {};
  }
  return (*std::get<realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>>>(
               this->inputs_.at(name).buffer)
               .readFromNonRT())
      ->data;
}

template<>
inline std::optional<std::vector<double>>
ModuloControllerInterface::read_input<std::vector<double>>(const std::string& name) {
  if (!check_input_valid(name)) {
    return {};
  }
  return (*std::get<realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>>>(
               this->inputs_.at(name).buffer)
               .readFromNonRT())
      ->data;
}

template<>
inline std::optional<int> ModuloControllerInterface::read_input<int>(const std::string& name) {
  if (!check_input_valid(name)) {
    return {};
  }
  return (*std::get<realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Int32>>>(
               this->inputs_.at(name).buffer)
               .readFromNonRT())
      ->data;
}

template<>
inline std::optional<std::string> ModuloControllerInterface::read_input<std::string>(const std::string& name) {
  if (!check_input_valid(name)) {
    return {};
  }
  return (*std::get<realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::String>>>(
               this->inputs_.at(name).buffer)
               .readFromNonRT())
      ->data;
}

template<typename T>
inline void ModuloControllerInterface::write_output(const std::string& name, const T& data) {
  if (data.is_empty()) {
    RCLCPP_DEBUG_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Skipping publication of output '%s' due to emptiness of state", name.c_str());
    return;
  }
  if (outputs_.find(name) == outputs_.end()) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000, "Could not find output '%s'", name.c_str());
    return;
  }
  EncodedStatePublishers publishers;
  try {
    publishers = std::get<EncodedStatePublishers>(outputs_.at(name));
  } catch (const std::bad_variant_access&) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Could not retrieve publisher for output '%s': Invalid output type", name.c_str());
    return;
  }
  if (const auto output_type = std::get<0>(publishers)->get_type(); output_type != data.get_type()) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Skipping publication of output '%s' due to wrong data type (expected '%s', got '%s')",
        state_representation::get_state_type_name(output_type).c_str(),
        state_representation::get_state_type_name(data.get_type()).c_str(), name.c_str());
    return;
  }
  auto rt_pub = std::get<2>(publishers);
  if (rt_pub && rt_pub->trylock()) {
    try {
      modulo_core::translators::write_message<T>(rt_pub->msg_, data, get_node()->get_clock()->now());
    } catch (const modulo_core::exceptions::MessageTranslationException& ex) {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000, "Failed to publish output '%s': %s", name.c_str(),
          ex.what());
    }
    rt_pub->unlockAndPublish();
  }
}

template<typename PublisherT, typename MsgT, typename T>
void ModuloControllerInterface::write_std_output(const std::string& name, const T& data) {
  if (outputs_.find(name) == outputs_.end()) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000, "Could not find output '%s'", name.c_str());
    return;
  }
  PublisherT publishers;
  try {
    publishers = std::get<PublisherT>(outputs_.at(name));
  } catch (const std::bad_variant_access&) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Could not retrieve publisher for output '%s': Invalid output type", name.c_str());
    return;
  }
  auto rt_pub = publishers.second;
  if (rt_pub && rt_pub->trylock()) {
    rt_pub->msg_.data = data;
    rt_pub->unlockAndPublish();
  }
}

template<>
inline void ModuloControllerInterface::write_output(const std::string& name, const bool& data) {
  write_std_output<BoolPublishers, std_msgs::msg::Bool, bool>(name, data);
}

template<>
inline void ModuloControllerInterface::write_output(const std::string& name, const double& data) {
  write_std_output<DoublePublishers, std_msgs::msg::Float64, double>(name, data);
}

template<>
inline void ModuloControllerInterface::write_output(const std::string& name, const std::vector<double>& data) {
  write_std_output<DoubleVecPublishers, std_msgs::msg::Float64MultiArray, std::vector<double>>(name, data);
}

template<>
inline void ModuloControllerInterface::write_output(const std::string& name, const int& data) {
  write_std_output<IntPublishers, std_msgs::msg::Int32, int>(name, data);
}

template<>
inline void ModuloControllerInterface::write_output(const std::string& name, const std::string& data) {
  write_std_output<StringPublishers, std_msgs::msg::String, std::string>(name, data);
}

}// namespace modulo_controller_interface
