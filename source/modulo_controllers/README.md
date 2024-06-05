# modulo-controllers

ROS 2 control interface classes for controllers in the AICA framework.

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

### Joints and control type

On top of any other individually added state interfaces, the controller will claim state interfaces matching the named
`joints`, and read this state data into a JointState object.

It will claim individual command interfaces from the hardware interface according to the named
`joints` and the specified `control_type` given to the class at construction. The `hardware_name` parameter determines
the name of the JointState object.

This class is intended to be used as a base class to implement joint and task space controllers. They can set the
joint command or access the joint state using the `set_joint_command()` and `get_joint_state()` methods, respectively.

### Robot model and URDF

If a controller needs a robot model, a `robot_model::Model` will be configured from URDF information. To support this,
the `robot_description` must be specified.

The robot model is used to calculate the CartesianState of a task frame from the JointState using forward kinematics,
which is available to derived classes using the `get_cartesian_state()` method.

The task frame can be specified using the `task_space_frame` parameter. If left empty, the final link of the
robot model will be used.

In certain cases, the `joints` parameter might not be in a physically correct order; for example, they might be sorted
alphabetically, which makes the corresponding JointState incompatible with the actual order of joints in a robot system.
If set to true, the `sort_joints` parameter orders the joint names to be physically correct using the robot model data.

Derived controllers can also access and leverage the robot model for more advanced behaviors such as inverse kinematics.

### Force-Torque Sensor

If the `ft_sensor_name` and `ft_sensor_reference_frame` parameters are set, the controller will look for a matching
sensor with the following state interfaces:

- `force.x`
- `force.y`
- `force.z`
- `torque.x`
- `torque.y`
- `torque.z`

Derived controllers can use `get_ft_sensor()` to access a CartesianWrench object of the sensor data. If the
`ft_sensor_reference_frame` matches the `task_space_frame`, the measured wrench will additionally be transformed to
the robot base frame and applied to the Cartesian state of the robot that is available through `get_cartesian_state()`.

### General behavior

The `command_half_life` parameter is used to attenuate any dynamic commands (velocities, accelerations or torques) in
case no new commands are received (i.e. if `set_joint_command()` was not called since the last controller update).

At each control step, the last command is scaled by a decay factor to cause an exponential fall-off towards zero. The
decay factor is calculated in terms of the desired command half-life based on the control frequency.
At the half-life time, the command value will be reduced by half. For example, a controller with a command half-life
of 0.1 seconds will have the output command reduced by 50%, 25%, 12.5% and 6.25% at 0.1, 0.2, 0.3 and 0.4 seconds
respectively.

The `command_rate_limit` parameter sets an upper bound on how much a command interface value can change at each
control step. It is expressed in units per second. For example, a controller with a rate limit of 5.0 commanding the
joint velocity interface at 100Hz (a control period of 0.01 seconds) will only allow the joint velocity to change by
up to 0.05 radians per second at each control step, or at 5.0 radians per second, per second.
