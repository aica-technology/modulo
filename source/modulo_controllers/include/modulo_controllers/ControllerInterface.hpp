#pragma once

#include "modulo_controllers/BaseControllerInterface.hpp"

namespace modulo_controllers {

/**
 * @class ControllerInterface
 */
class ControllerInterface : public BaseControllerInterface {
public:
  /**
   * @brief Default constructor
   * @param claim_all_state_interfaces Flag to indicate if all state interfaces should be claimed
   */
  ControllerInterface(bool claim_all_state_interfaces = false);

  /**
   * @brief Declare parameters
   * @return CallbackReturn status
   */
  CallbackReturn on_init() override;

  /**
   * @brief Set class properties from parameters.
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
   * @brief Add interfaces like parameters, signals, services, and predicates to the controller.
   * @details This function is called during the `on_init` callback of the base class to perform post-construction
   * steps.
   * @return SUCCESS or ERROR
  */
  virtual CallbackReturn add_interfaces();

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

  using controller_interface::ControllerInterfaceBase::command_interfaces_;
  using controller_interface::ControllerInterfaceBase::state_interfaces_;

  std::unordered_map<std::string, std::unordered_map<std::string, double>>
      state_interface_data_;///< Map of all state interface data
  std::vector<double> command_interface_data_;///< Vector of all command interface data
  std::unordered_map<std::string, std::unordered_map<std::string, unsigned int>>
      command_interface_indices_;                   ///< Map of command interface indices in the command interface data
  std::vector<std::string> state_interface_names_;  ///< List of state interfaces to claim
  std::vector<std::string> command_interface_names_;///< List of command interfaces to claim
  bool claim_all_state_interfaces_;                 ///< Flag to decide if all state interfaces are claimed

  // TODO make missed_locks an internal parameter
  unsigned int missed_locks_;
  bool on_init_called_;
};

}// namespace modulo_controllers
