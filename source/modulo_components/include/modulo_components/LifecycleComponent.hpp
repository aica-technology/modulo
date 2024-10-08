#pragma once

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "modulo_components/ComponentInterface.hpp"

namespace modulo_components {

/**
 * @class LifecycleComponent
 * @brief A wrapper for rclcpp_lifecycle::LifecycleNode to simplify application composition through unified component
 * interfaces while supporting lifecycle states and transitions.
 * @details This class is intended for direct inheritance to implement custom state-based components that perform
 * different behaviors based on their state and on state transitions. An example of state-based behaviour is a
 * signal component that requires configuration steps to determine which inputs to register and subsequently should
 * publish outputs only when the component is activated.
 * Developers should override on_validate_parameter_callback() if any parameters are added. In addition, the following
 * state transition callbacks should be overridden whenever custom transition behavior is needed:
 * - on_configure_callback()
 * - on_activate_callback()
 * - on_deactivate_callback()
 * - on_cleanup_callback()
 * - on_shutdown_callback()
 * - on_error_callback()
 * - on_step_callback()
 * @see Component for a stateless composition alternative
 */
class LifecycleComponent : public rclcpp_lifecycle::LifecycleNode, public ComponentInterface {
public:
  friend class LifecycleComponentPublicInterface;

  /**
   * @brief Constructor from node options.
   * @param node_options Node options as used in ROS2 LifecycleNode
   * @param fallback_name The name of the component if it was not provided through the node options
   */
  explicit LifecycleComponent(
      const rclcpp::NodeOptions& node_options, const std::string& fallback_name = "LifecycleComponent");

  /**
   * @brief Virtual default destructor.
   */
  virtual ~LifecycleComponent() = default;

protected:
  /**
   * @copydoc ComponentInterface::get_parameter
   */
  [[nodiscard]] std::shared_ptr<state_representation::ParameterInterface> get_parameter(const std::string& name) const;

  /**
   * @brief Steps to execute when configuring the component.
   * @details This method can be overridden by derived Component classes.
   * Configuration generally involves reading parameters and adding inputs and outputs.
   * @return True if configuration is successful, false otherwise
   */
  virtual bool on_configure_callback();

  /**
   * @brief Steps to execute when cleaning up the component.
   * @details This method can be overridden by derived Component classes.
   * Cleanup generally involves resetting the properties and states to initial conditions.
   * @return True if cleanup is successful, false otherwise
   */
  virtual bool on_cleanup_callback();

  /**
   * @brief Steps to execute when activating the component.
   * @details This method can be overridden by derived Component classes.
   * Activation generally involves final setup steps before the on_step callback is periodically evaluated.
   * @return True if activation is successful, false otherwise
   */
  virtual bool on_activate_callback();

  /**
   * @brief Steps to execute when deactivating the component.
   * @details This method can be overridden by derived Component classes.
   * Deactivation generally involves any steps to reset the component to an inactive state.
   * @return True if deactivation is successful, false otherwise
   */
  virtual bool on_deactivate_callback();

  /**
   * @brief Steps to execute when shutting down the component.
   * @details This method can be overridden by derived Component classes.
   * Shutdown generally involves the destruction of any threads or properties not handled by the base class.
   * @return True if shutdown is successful, false otherwise
   */
  virtual bool on_shutdown_callback();

  /**
   * @brief Steps to execute when handling errors.
   * @details This method can be overridden by derived Component classes.
   * Error handling generally involves recovering and resetting the component to an unconfigured state.
   * @return True if error handling is successful, false otherwise
   */
  virtual bool on_error_callback();

  /**
   * @brief Add an output signal of the component.
   * @tparam DataT Type of the data pointer
   * @param signal_name Name of the output signal
   * @param data Data to transmit on the output signal
   * @param default_topic If set, the default value for the topic name to use
   * @param fixed_topic If true, the topic name of the output signal is fixed
   * @param publish_on_step If true, the output is published periodically on step
   */
  template<typename DataT>
  void add_output(
      const std::string& signal_name, const std::shared_ptr<DataT>& data, const std::string& default_topic = "",
      bool fixed_topic = false, bool publish_on_step = true);

  /**
   * @brief Get the current lifecycle state of the component
   */
  rclcpp_lifecycle::State get_lifecycle_state() const;

  /**
   * @brief Trigger the shutdown and error transitions.
   */
  void raise_error() override;

private:
  /**
   * @brief Transition callback for state 'Configuring'.
   * @details on_configure callback is called when the lifecycle component enters the 'Configuring' transition state.
   * The component must be in the 'Unconfigured' state.
   * Depending on the return value of this function, the component may either transition to the 'Inactive' state
   * via the 'configure' transition, stay 'Unconfigured' or go to 'ErrorProcessing'.\n
   * TRANSITION_CALLBACK_SUCCESS transitions to 'Inactive'\n
   * TRANSITION_CALLBACK_FAILURE transitions to 'Unconfigured'\n
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'\n
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Configuration handler.
   * @details This method configures outputs and invokes the virtual on_configure callback.
   * @return True if configuration is successful, false otherwise
   */
  bool handle_configure();

  /**
    * @brief Transition callback for state 'CleaningUp'.
    * @details on_cleanup callback is called when the lifecycle component enters the 'CleaningUp' transition state.
    * The component must be in the 'Inactive' state.
    * Depending on the return value of this function, the component may either transition to the 'Unconfigured' state
    * via the 'cleanup' transition or go to 'ErrorProcessing'.\n
    * TRANSITION_CALLBACK_SUCCESS transitions to 'Unconfigured'\n
    * TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'\n
    */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Cleanup handler.
   * @details This method resets inputs and outputs and invokes the virtual on_cleanup callback.
   * @return True if cleanup is successful, false otherwise
   */
  bool handle_cleanup();

  /**
  * @brief Transition callback for state 'Activating'.
  * @details on_activate callback is called when the lifecycle component enters the 'Activating' transition state.
  * The component must be in the 'Inactive' state.
  * Depending on the return value of this function, the component may either transition to the 'Active' state
  * via the 'activate' transition, stay 'Inactive' or go to 'ErrorProcessing'.\n
  * TRANSITION_CALLBACK_SUCCESS transitions to 'Active'\n
  * TRANSITION_CALLBACK_FAILURE transitions to 'Inactive'\n
  * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'\n
  */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Activation handler.
   * @details This method activates outputs and invokes the virtual on_activate callback.
   * @return True if activation is successful, false otherwise
   */
  bool handle_activate();

  /**
    * @brief Transition callback for state 'Deactivating'.
    * @details on_deactivate callback is called when the lifecycle component enters the 'Deactivating' transition state.
    * The component must be in the 'Active' state.
    * Depending on the return value of this function, the component may either transition to the 'Inactive' state
    * via the 'deactivate' transition or go to 'ErrorProcessing'.\n
    * TRANSITION_CALLBACK_SUCCESS transitions to 'Inactive'\n
    * TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'\n
    */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Deactivation handler.
   * @details This method deactivates outputs and invokes the virtual on_deactivate callback.
   * @return True if deactivation is successful, false otherwise
   */
  bool handle_deactivate();

  /**
    * @brief Transition callback for state 'ShuttingDown'.
    * @details on_shutdown callback is called when the lifecycle component enters the 'ShuttingDown' transition state.
    * This transition can be called from the 'Unconfigured', 'Inactive' and 'Active' states.
    * Depending on the return value of this function, the component may either transition to the 'Finalized' state
    * via the 'shutdown' transition or go to 'ErrorProcessing'.\n
    * TRANSITION_CALLBACK_SUCCESS transitions to 'Finalized'\n
    * TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'\n
    */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Shutdown handler.
   * @details This method invokes the virtual on_shutdown callback.
   * @return True if shutdown is successful, false otherwise
   */
  bool handle_shutdown();

  /**
    * @brief Transition callback for state 'ErrorProcessing'.
    * @details on_error callback is called when the lifecycle component enters the 'ErrorProcessing' transition state.
    * This transition can originate from any step.
    * Depending on the return value of this function, the component may either transition to the 'Unconfigured' state
    * or go to 'Finalized'.\n
    * TRANSITION_CALLBACK_SUCCESS transitions to 'Unconfigured'\n
    * TRANSITION_CALLBACK_FAILURE transitions to 'Finalized'\n
    * TRANSITION_CALLBACK_ERROR should not be returned, and any exceptions should be caught and returned as a failure\n
    */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Error handler.
   * @details This method invokes the virtual on_error callback.
   * @return True if error handling is successful, false otherwise
   */
  bool handle_error();

  /**
   * @brief Step function that is called periodically and publishes predicates,
   * outputs, evaluates daemon callbacks, and calls the on_step function.
   */
  void step() override;

  /**
   * @brief Configure all outputs.
   * @return True if configuration was successful
   */
  bool configure_outputs();

  /**
   * @brief Activate all outputs.
   * @return True if activation was successful
   */
  bool activate_outputs();

  /**
   * @brief Deactivate all outputs.
   * @return True if deactivation was successful
   */
  bool deactivate_outputs();

  bool has_error_;

  // TODO hide ROS methods
  using ComponentInterface::create_output;
  using ComponentInterface::evaluate_periodic_callbacks;
  using ComponentInterface::finalize_interfaces;
  using ComponentInterface::get_parameter;
  using ComponentInterface::inputs_;
  using ComponentInterface::outputs_;
  using ComponentInterface::periodic_outputs_;
  using ComponentInterface::publish_outputs;
  using ComponentInterface::publish_predicates;
  using rclcpp_lifecycle::LifecycleNode::get_parameter;

  std::map<
      std::string,
      std::function<std::shared_ptr<modulo_core::communication::PublisherInterface>(const std::string& topic_name)>>
      custom_output_configuration_callables_;///< Map of custom output configuration callables
};

template<typename DataT>
inline void LifecycleComponent::add_output(
    const std::string& signal_name, const std::shared_ptr<DataT>& data, const std::string& default_topic,
    bool fixed_topic, bool publish_on_step) {
  if (this->get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED
      && this->get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Adding output in state " << this->get_lifecycle_state().label() << " is not allowed.");
    return;
  }
  try {
    using modulo_core::communication::PublisherHandler;
    using modulo_core::communication::PublisherType;

    auto parsed_signal_name = this->create_output(
        PublisherType::LIFECYCLE_PUBLISHER, signal_name, data, default_topic, fixed_topic, publish_on_step);

    auto message_pair = this->outputs_.at(parsed_signal_name)->get_message_pair();
    if (message_pair->get_type() == modulo_core::communication::MessageType::CUSTOM_MESSAGE) {
      if constexpr (modulo_core::concepts::CustomT<DataT>) {
        this->custom_output_configuration_callables_.insert_or_assign(
            parsed_signal_name, [this, message_pair](const std::string& topic_name) {
              auto publisher = this->create_publisher<DataT>(topic_name, this->get_qos());
              return std::make_shared<PublisherHandler<rclcpp_lifecycle::LifecyclePublisher<DataT>, DataT>>(
                         PublisherType::LIFECYCLE_PUBLISHER, publisher)
                  ->create_publisher_interface(message_pair);
            });
      }
    }
  } catch (const modulo_core::exceptions::AddSignalException& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to add output '" << signal_name << "': " << ex.what());
  }
}
}// namespace modulo_components
