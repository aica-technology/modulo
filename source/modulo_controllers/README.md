# modulo-controllers

ROS2 control interface class for controllers based on the control libraries stack.

## ControllerInterface

This C++ class derives from ros2_control `ControllerInterface` and incorporates `modulo` concepts like inputs, outputs,
parameters, predicates, and services.

It supports the following parameters:

- `hardware_name` [string]: the name of the hardware interface
- `robot_description` [string]: the string formatted content of the controller's URDF description
- `joints` [string_array]: a vector of joint names that the controller will claim
- `activation_timeout` [double]: the seconds to wait for valid data on state interfaces before activating
- `input_validity_period` [double]: the maximum age of an input state before discarding it as expired

### General behavior

The controller will claim either all or individually added state interfaces and individually added command
interfaces. The class is intended to be used as a base class to implement any kind of controller.

The `activation_timeout` parameter gives the controller plugin and hardware interface additional time to read the
initial states to prevent any NaN data from propagating through to the control logic.

## RobotControllerInterface

The `RobotControllerInterface` is derived from the `ControllerInterface`. It incorporates `JointState` and
`CartesianState` classes to leverage the `robot_model::Model` class for forward and inverse kinematics, and supports
force-torque sensor data in `CartesianWrench` format.
