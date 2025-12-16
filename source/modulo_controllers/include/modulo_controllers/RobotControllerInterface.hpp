#pragma once

#include <realtime_tools/realtime_buffer.h>

#include <robot_model/Model.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include "modulo_controllers/ControllerInterface.hpp"

namespace modulo_controllers {

/**
 * @class RobotControllerInterface
 * @brief Base controller class that automatically associates joints with a JointState object
 * @details The robot controller interface extends the functionality of the modulo controller interface by automatically
 * claiming all state interfaces from joints and command interfaces of a given type (position, velocity, effort or
 * acceleration) for those same joints. Joint state and command are associated with these interfaces and abstracted as
 * JointState pointers for derived classes to access. A robot model, Cartesian state and force-torque sensor state are
 * similarly available based on the URDF and state interfaces.
 */
class RobotControllerInterface : public ControllerInterface {
public:
  /**
   * @brief Default constructor
   */
  RobotControllerInterface();

  /**
   * @brief Constructor with robot model flag and a control type to determine the command interfaces to claim
   * @param robot_model_required Flag to indicate if a robot model is required for the controller
   * @param control_type One of [position, velocity, effort or acceleration]
   * @param load_geometries If true, load the URDF geometries into the robot model for collision features
   */
  explicit RobotControllerInterface(
      bool robot_model_required, const std::string& control_type = "", bool load_geometries = false);

  /**
   * @copydoc modulo_controllers::ControllerInterface::add_interfaces()
   * @details Declare additional parameters.
   */
  CallbackReturn add_interfaces() override;

  /**
   * @copydoc modulo_controllers::ControllerInterface::on_configure()
   * @details Create a robot model from the robot description, get and sort the joints and construct the internal joint
   * state object.
   */
  CallbackReturn on_configure() override;

  /**
   * @copydoc modulo_controllers::ControllerInterface::on_activate()
   * @details Initialize a fore torque sensor if applicable
   */
  CallbackReturn on_activate() override;

  /**
   * @copydoc modulo_controllers::ControllerInterface::on_deactivate()
   * @details Reset the previous joint commands
   */
  CallbackReturn on_deactivate() override;

protected:
  /**
   * @brief Access the joint state object.
   * @return A const reference to the JointState object
   */
  const state_representation::JointState& get_joint_state();

  /**
   * @brief Access the Cartesian state object.
   * @details This internally calls compute_cartesian_state()
   * @return A const reference to the CartesianState object
   */
  const state_representation::CartesianState& get_cartesian_state();

  /**
   * @brief Access the Cartesian wrench object.
   * @return A const reference to the CartesianWrench object
   */
  const state_representation::CartesianWrench& get_ft_sensor();

  /**
   * @brief Compute the Cartesian state from forward kinematics of the current joint state.
   * @details This should only be used if a robot model has been generated, in which case
   * the forward kinematics is calculated to get pose and twist for the desired target frame.
   */
  void compute_cartesian_state();

  /**
   * @brief Set the joint command object.
   * @param joint_command A JointState command object
   */
  void set_joint_command(const state_representation::JointState& joint_command);

  /**
   * @copydoc modulo_controllers::ControllerInterface::on_validate_parameter_callback()
   */
  bool
  on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;

  /**
   * @brief Get the control type of the controller, empty or one of [position, velocity, effort or acceleration]
   */
  std::string get_control_type() const;

  /**
   * @brief Set the control type of the controller after construction
   * @param control_type One of [position, velocity, effort or acceleration]
   * @throws std::runtime_error if the control type is already fixed (after configuring) or invalid
   */
  void set_control_type(const std::string& control_type);

  std::shared_ptr<robot_model::Model> robot_;///< Robot model object generated from URDF
  std::string task_space_frame_;///< The frame in task space for forward kinematics calculations, if applicable

  std::string ft_sensor_name_;           ///< The name of a force torque sensor in the hardware description
  std::string ft_sensor_reference_frame_;///< The sensing reference frame

private:
  /**
   * @copydoc modulo_controllers::ControllerInterface::read_state_interfaces()
   * @details Update the internal joint state and force torque sensor objects from the new state interface values.
   */
  controller_interface::return_type read_state_interfaces() final;

  /**
   * @copydoc modulo_controllers::ControllerInterface::write_command_interfaces()
   * @details Apply rate limiting and command decay to the buffered command to update the command interfaces.
   */
  controller_interface::return_type write_command_interfaces(const rclcpp::Duration& period) final;

  std::vector<std::string> joints_;///< The joint names provided by a parameter
  std::string control_type_;       ///< The high-level interface type (position, velocity, acceleration or effort)
  bool control_type_fixed_;        ///< If true, the control type cannot be changed after

  bool robot_model_required_;///< If true, check that a robot model is available on configure
  bool load_geometries_;     ///< If true, load geometries from the URDF into the robot model
  bool
      new_joint_command_ready_;///< If true, joint commands are written to the command interface, else previous commands are decayed
  double
      command_decay_factor_;///< Dimensionless decay factor calculated from the desired command half life, to be scaled by controller timestep
  double
      command_rate_limit_;///< Maximum value change of a command interface per second, to be scaled by controller timestep

  realtime_tools::RealtimeBuffer<std::vector<double>> joint_command_values_;
  std::vector<double> previous_joint_command_values_;

  state_representation::JointState joint_state_;
  state_representation::CartesianState cartesian_state_;
  state_representation::CartesianWrench ft_sensor_;
};

}// namespace modulo_controllers
