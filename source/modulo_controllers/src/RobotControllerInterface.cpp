#include "modulo_controllers/RobotControllerInterface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <state_representation/exceptions/JointNotFoundException.hpp>

using namespace state_representation;

namespace modulo_controllers {

static const std::map<std::string, JointStateVariable> interface_map =// NOLINT(cert-err58-cpp)
    {{hardware_interface::HW_IF_POSITION, JointStateVariable::POSITIONS},
     {hardware_interface::HW_IF_VELOCITY, JointStateVariable::VELOCITIES},
     {hardware_interface::HW_IF_ACCELERATION, JointStateVariable::ACCELERATIONS},
     {hardware_interface::HW_IF_EFFORT, JointStateVariable::TORQUES}};

RobotControllerInterface::RobotControllerInterface() : RobotControllerInterface(false, "", false) {}

RobotControllerInterface::RobotControllerInterface(
    bool robot_model_required, const std::string& control_type, bool load_geometries)
    : ControllerInterface(true),
      control_type_(control_type),
      robot_model_required_(robot_model_required),
      load_geometries_(load_geometries),
      new_joint_command_ready_(false),
      command_decay_factor_(0.0),
      command_rate_limit_(std::numeric_limits<double>::infinity()) {
  if (!control_type.empty() && interface_map.find(control_type) == interface_map.cend()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid control type: %s", control_type.c_str());
    throw std::invalid_argument("Invalid control type");
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControllerInterface::add_interfaces() {
  add_parameter(
      std::make_shared<Parameter<std::string>>("task_space_frame"),
      "The frame name in the robot model to use for kinematics calculations (defaults to the last frame in the "
      "model)");
  add_parameter<bool>(
      "sort_joints", true,
      "If true, re-arrange the 'joints' parameter into a physically correct order according to the robot model");
  add_parameter(
      std::make_shared<Parameter<std::string>>("ft_sensor_name"),
      "Optionally, the name of a force-torque sensor in the hardware interface");
  add_parameter(
      std::make_shared<Parameter<std::string>>("ft_sensor_reference_frame"),
      "The reference frame of the force-torque sensor in the robot model");
  add_parameter<double>(
      "command_half_life", 0.1,
      "A time constant for the exponential decay of the commanded velocity, acceleration or torque if no new command "
      "is set");
  add_parameter<double>(
      "command_rate_limit", command_rate_limit_,
      "The maximum allowable change in command on any interface expressed in command units / second");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControllerInterface::on_configure() {
  std::stringstream timestamp;
  timestamp << std::time(nullptr);
  auto urdf_path = "/tmp/" + hardware_name_ + "_" + timestamp.str();
  try {
    robot_model::Model::create_urdf_from_string(get_robot_description(), urdf_path);
    if (load_geometries_) {
      robot_ = std::make_shared<robot_model::Model>(hardware_name_, urdf_path, [](const std::string& package_name) {
        return ament_index_cpp::get_package_share_directory(package_name) + "/";
      });
      RCLCPP_DEBUG(get_node()->get_logger(), "Generated robot model with collision features");
    } else {
      robot_ = std::make_shared<robot_model::Model>(hardware_name_, urdf_path);
      RCLCPP_DEBUG(get_node()->get_logger(), "Generated robot model");
    }
  } catch (const std::exception& ex) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Could not generate robot model with temporary urdf from string content at path %s: %s", urdf_path.c_str(),
        ex.what());
  }
  if (robot_model_required_ && robot_ == nullptr) {
    RCLCPP_ERROR(get_node()->get_logger(), "Robot model is not available even though it's required by the controller.");
    return CallbackReturn::ERROR;
  }

  if (robot_) {
    if (get_parameter("task_space_frame")->is_empty()) {
      task_space_frame_ = robot_->get_frames().back();
    } else {
      task_space_frame_ = get_parameter_value<std::string>("task_space_frame");
      if (!robot_->get_pinocchio_model().existFrame(task_space_frame_)) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Provided task space frame %s does not exist in the robot model!",
            task_space_frame_.c_str());
        return CallbackReturn::ERROR;
      }
    }
  }

  auto joints = get_parameter("joints");
  if (joints->is_empty() && !control_type_.empty()) {
    RCLCPP_ERROR(
        get_node()->get_logger(), "The 'joints' parameter is required for a non-zero control type %s!",
        control_type_.c_str());
    return CallbackReturn::ERROR;
  }
  joints_ = joints->get_parameter_value<std::vector<std::string>>();

  // sort the interface joint names using the robot model if necessary
  if (get_parameter_value<bool>("sort_joints") && robot_) {
    if (joints_.size() != robot_->get_number_of_joints()) {
      RCLCPP_WARN(
          get_node()->get_logger(),
          "The number of interface joints (%zu) does not match the number of joints in the robot model (%u)",
          joints_.size(), robot_->get_number_of_joints());
    }
    std::vector<std::string> ordered_joints;
    for (const auto& ordered_name : robot_->get_joint_frames()) {
      for (const auto& joint_name : joints_) {
        if (joint_name == ordered_name) {
          ordered_joints.push_back(joint_name);
          break;
        }
      }
    }
    if (ordered_joints.size() < joints_.size()) {
      RCLCPP_ERROR(
          get_node()->get_logger(), "%zu interface joints were not found in the robot model",
          joints_.size() - ordered_joints.size());
      return CallbackReturn::ERROR;
    }
    joints_ = ordered_joints;
  }
  joint_state_ = JointState(hardware_name_, joints_);

  // set command interfaces from joints
  if (!control_type_.empty()) {
    if (control_type_ == hardware_interface::HW_IF_POSITION) {
      previous_joint_command_values_ = std::vector<double>(joints_.size(), std::numeric_limits<double>::quiet_NaN());
    } else {
      previous_joint_command_values_ = std::vector<double>(joints_.size(), 0.0);
    }
    for (const auto& joint : joints_) {
      add_command_interface(joint, control_type_);
    }
  }

  auto ft_sensor_name = get_parameter("ft_sensor_name");
  if (!ft_sensor_name->is_empty()) {
    ft_sensor_name_ = ft_sensor_name->get_parameter_value<std::string>();
    auto ft_sensor_reference_frame = get_parameter("ft_sensor_reference_frame");
    if (ft_sensor_reference_frame->is_empty()) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "The 'ft_sensor_reference_frame' parameter cannot be empty if a force-torque sensor is specified!");
      return CallbackReturn::ERROR;
    }
    ft_sensor_reference_frame_ = ft_sensor_reference_frame->get_parameter_value<std::string>();
    // TODO check that there is no joint in between
    if (robot_ != nullptr && !robot_->get_pinocchio_model().existFrame(ft_sensor_reference_frame_)) {
      RCLCPP_ERROR(
          get_node()->get_logger(), "The FT sensor reference frame '%s' does not exist on the robot model!",
          ft_sensor_reference_frame_.c_str());
      return CallbackReturn::ERROR;
    }
  }

  command_decay_factor_ = -log(0.5) / get_parameter_value<double>("command_half_life");
  command_rate_limit_ = get_parameter_value<double>("command_rate_limit");

  RCLCPP_DEBUG(get_node()->get_logger(), "Configuration of RobotControllerInterface successful");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotControllerInterface::on_activate() {
  // initialize a force torque sensor state if applicable
  if (!ft_sensor_name_.empty()) {
    try {
      get_state_interface(ft_sensor_name_, "force.x");
      get_state_interface(ft_sensor_name_, "force.y");
      get_state_interface(ft_sensor_name_, "force.z");
      get_state_interface(ft_sensor_name_, "torque.x");
      get_state_interface(ft_sensor_name_, "torque.y");
      get_state_interface(ft_sensor_name_, "torque.z");
    } catch (const std::out_of_range&) {
      RCLCPP_ERROR(
          get_node()->get_logger(), "The force torque sensor '%s' does not have all the expected state interfaces!",
          ft_sensor_name_.c_str());
      return CallbackReturn::FAILURE;
    }
    ft_sensor_ = CartesianWrench(ft_sensor_name_, ft_sensor_reference_frame_);
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "Activation of RobotControllerInterface successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotControllerInterface::read_state_interfaces() {
  auto status = ControllerInterface::read_state_interfaces();
  if (status != controller_interface::return_type::OK) {
    return status;
  }

  if (joint_state_.get_size() > 0 && joint_state_.is_empty()) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Reading first joint state");
    joint_state_.set_zero();
  }

  for (const auto& joint : joints_) {
    auto data = get_state_interfaces(joint);
    unsigned int joint_index;
    try {
      joint_index = joint_state_.get_joint_index(joint);
    } catch (exceptions::JointNotFoundException& ex) {
      RCLCPP_ERROR_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, ex.what());
      return controller_interface::return_type::ERROR;
    }
    for (const auto& [interface, value] : data) {
      if (!std::isfinite(value)) {
        RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000, "Value of state interface '%s/%s' is not finite",
            joint.c_str(), interface.c_str());
        return controller_interface::return_type::ERROR;
      }
      switch (interface_map.at(interface)) {
        case JointStateVariable::POSITIONS:
          joint_state_.set_position(value, joint_index);
          break;
        case JointStateVariable::VELOCITIES:
          joint_state_.set_velocity(value, joint_index);
          break;
        case JointStateVariable::ACCELERATIONS:
          joint_state_.set_acceleration(value, joint_index);
          break;
        case JointStateVariable::TORQUES:
          joint_state_.set_torque(value, joint_index);
          break;
        default:
          break;
      }
    }
  }

  if (!ft_sensor_name_.empty()) {
    const auto& ft_sensor = get_state_interfaces(ft_sensor_name_);
    std::vector<double> wrench = {ft_sensor.at("force.x"),  ft_sensor.at("force.y"),  ft_sensor.at("force.z"),
                                  ft_sensor.at("torque.x"), ft_sensor.at("torque.y"), ft_sensor.at("torque.z")};
    ft_sensor_.set_wrench(wrench);
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type RobotControllerInterface::write_command_interfaces(const rclcpp::Duration& period) {
  if (!control_type_.empty()) {
    double decay = 1.0 - command_decay_factor_ * period.seconds();
    decay = std::max(0.0, decay);
    double rate_limit = command_rate_limit_ * period.seconds();

    auto joint_command_values = joint_command_values_.readFromRT();
    if (joint_command_values == nullptr) {
      new_joint_command_ready_ = false;
    }

    for (std::size_t index = 0; index < joints_.size(); ++index) {
      double previous_command = previous_joint_command_values_.at(index);
      if (new_joint_command_ready_) {
        auto new_command = joint_command_values->at(index);
        if (std::isfinite(previous_command) && std::abs(new_command - previous_command) > rate_limit) {
          double command_offset = new_command - previous_command > 0.0 ? rate_limit : -rate_limit;
          new_command = previous_command + command_offset;
        }
        set_command_interface(joints_.at(index), control_type_, new_command);
        previous_joint_command_values_.at(index) = new_command;
      } else if (control_type_ != hardware_interface::HW_IF_POSITION) {
        auto new_command = previous_command * decay;
        set_command_interface(joints_.at(index), control_type_, new_command);
        previous_joint_command_values_.at(index) = new_command;
      }
    }
    new_joint_command_ready_ = false;
  }

  return ControllerInterface::write_command_interfaces(period);
}

const JointState& RobotControllerInterface::get_joint_state() {
  return joint_state_;
}

const CartesianState& RobotControllerInterface::get_cartesian_state() {
  // TODO check if recompute is necessary?
  compute_cartesian_state();
  return cartesian_state_;
}

const CartesianWrench& RobotControllerInterface::get_ft_sensor() {
  return ft_sensor_;
}

void RobotControllerInterface::compute_cartesian_state() {
  if (robot_ != nullptr) {
    cartesian_state_ = robot_->forward_kinematics(joint_state_, task_space_frame_);
    cartesian_state_.set_twist(robot_->forward_velocity(joint_state_, task_space_frame_).get_twist());

    if (!ft_sensor_.is_empty() && (ft_sensor_.get_reference_frame() == cartesian_state_.get_name())) {
      auto ft_sensor_in_robot_frame = cartesian_state_ * ft_sensor_;
      cartesian_state_.set_wrench(ft_sensor_in_robot_frame.get_wrench());
    }
  }
}

void RobotControllerInterface::set_joint_command(const JointState& joint_command) {
  if (!joint_command) {
    RCLCPP_DEBUG(get_node()->get_logger(), "set_joint_command called with an empty JointState");
    return;
  }

  std::vector<double> joint_command_values;
  joint_command_values.reserve(joints_.size());

  for (const auto& joint : joints_) {
    try {
      switch (interface_map.at(control_type_)) {
        case JointStateVariable::POSITIONS:
          joint_command_values.push_back(joint_command.get_position(joint));
          break;
        case JointStateVariable::VELOCITIES:
          joint_command_values.push_back(joint_command.get_velocity(joint));
          break;
        case JointStateVariable::ACCELERATIONS:
          joint_command_values.push_back(joint_command.get_acceleration(joint));
          break;
        case JointStateVariable::TORQUES:
          joint_command_values.push_back(joint_command.get_torque(joint));
          break;
        default:
          break;
      }
    } catch (const state_representation::exceptions::JointNotFoundException& ex) {
      RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, ex.what());
    }
  }

  if (joint_command_values.size() == joints_.size()) {
    joint_command_values_.writeFromNonRT(joint_command_values);
    new_joint_command_ready_ = true;
  } else {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Unable to set command values from provided joint command");
  }
}

bool RobotControllerInterface::on_validate_parameter_callback(const std::shared_ptr<ParameterInterface>& parameter) {
  if ((parameter->get_name() == "command_half_life" || parameter->get_name() == "command_rate_limit")
      && parameter->get_parameter_value<double>() < 0.0) {
    RCLCPP_ERROR(
        get_node()->get_logger(), "Parameter value of '%s' should be greater than 0", parameter->get_name().c_str());
    return false;
  }
  return true;
}

}// namespace modulo_controllers
