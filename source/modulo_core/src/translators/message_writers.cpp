#include "modulo_core/translators/message_writers.hpp"

#include <state_representation/space/cartesian/CartesianPose.hpp>

using namespace state_representation;

namespace modulo_core::translators {

static void write_point(geometry_msgs::msg::Point& message, const Eigen::Vector3d& vector) {
  message.x = vector.x();
  message.y = vector.y();
  message.z = vector.z();
}

static void write_vector3(geometry_msgs::msg::Vector3& message, const Eigen::Vector3d& vector) {
  message.x = vector.x();
  message.y = vector.y();
  message.z = vector.z();
}

static void write_quaternion(geometry_msgs::msg::Quaternion& message, const Eigen::Quaterniond& quat) {
  message.w = quat.w();
  message.x = quat.x();
  message.y = quat.y();
  message.z = quat.z();
}

void write_message(geometry_msgs::msg::Accel& message, const CartesianState& state, const rclcpp::Time&) {
  if (!state) {
    throw exceptions::MessageTranslationException(
        state.get_name() + " state is empty while attempting to write it to message");
  }
  write_vector3(message.linear, state.get_linear_acceleration());
  write_vector3(message.angular, state.get_angular_acceleration());
}

void write_message(geometry_msgs::msg::AccelStamped& message, const CartesianState& state, const rclcpp::Time& time) {
  write_message(message.accel, state, time);
  message.header.stamp = time;
  message.header.frame_id = state.get_reference_frame();
}

void write_message(geometry_msgs::msg::Pose& message, const CartesianState& state, const rclcpp::Time&) {
  if (!state) {
    throw exceptions::MessageTranslationException(
        state.get_name() + " state is empty while attempting to write it to message");
  }
  write_point(message.position, state.get_position());
  write_quaternion(message.orientation, state.get_orientation());
}

void write_message(geometry_msgs::msg::PoseStamped& message, const CartesianState& state, const rclcpp::Time& time) {
  write_message(message.pose, state, time);
  message.header.stamp = time;
  message.header.frame_id = state.get_reference_frame();
}

void write_message(geometry_msgs::msg::Transform& message, const CartesianState& state, const rclcpp::Time&) {
  if (!state) {
    throw exceptions::MessageTranslationException(
        state.get_name() + " state is empty while attempting to write it to message");
  }
  write_vector3(message.translation, state.get_position());
  write_quaternion(message.rotation, state.get_orientation());
}

void
write_message(geometry_msgs::msg::TransformStamped& message, const CartesianState& state, const rclcpp::Time& time) {
  write_message(message.transform, state, time);
  message.header.stamp = time;
  message.header.frame_id = state.get_reference_frame();
  message.child_frame_id = state.get_name();
}

void write_message(geometry_msgs::msg::Twist& message, const CartesianState& state, const rclcpp::Time&) {
  if (!state) {
    throw exceptions::MessageTranslationException(
        state.get_name() + " state is empty while attempting to write it to message");
  }
  write_vector3(message.linear, state.get_linear_velocity());
  write_vector3(message.angular, state.get_angular_velocity());
}

void write_message(geometry_msgs::msg::TwistStamped& message, const CartesianState& state, const rclcpp::Time& time) {
  write_message(message.twist, state, time);
  message.header.stamp = time;
  message.header.frame_id = state.get_reference_frame();
}

void write_message(geometry_msgs::msg::Wrench& message, const CartesianState& state, const rclcpp::Time&) {
  if (!state) {
    throw exceptions::MessageTranslationException(
        state.get_name() + " state is empty while attempting to write it to message");
  }
  write_vector3(message.force, state.get_force());
  write_vector3(message.torque, state.get_torque());
}

void write_message(geometry_msgs::msg::WrenchStamped& message, const CartesianState& state, const rclcpp::Time& time) {
  write_message(message.wrench, state, time);
  message.header.stamp = time;
  message.header.frame_id = state.get_reference_frame();
}

void write_message(sensor_msgs::msg::JointState& message, const JointState& state, const rclcpp::Time& time) {
  if (!state) {
    throw exceptions::MessageTranslationException(
        state.get_name() + " state is empty while attempting to write it to message");
  }
  message.header.stamp = time;
  message.name = state.get_names();
  message.position =
      std::vector<double>(state.get_positions().data(), state.get_positions().data() + state.get_positions().size());
  message.velocity =
      std::vector<double>(state.get_velocities().data(), state.get_velocities().data() + state.get_velocities().size());
  message.effort =
      std::vector<double>(state.get_torques().data(), state.get_torques().data() + state.get_torques().size());
}

void write_message(tf2_msgs::msg::TFMessage& message, const CartesianState& state, const rclcpp::Time& time) {
  if (!state) {
    throw exceptions::MessageTranslationException(
        state.get_name() + " state is empty while attempting to write it to message");
  }
  geometry_msgs::msg::TransformStamped transform;
  write_message(transform, state, time);
  message.transforms.push_back(transform);
}

template<typename U, typename T>
void write_message(U& message, const Parameter<T>& state, const rclcpp::Time&) {
  if (!state) {
    throw exceptions::MessageTranslationException(
        state.get_name() + " state is empty while attempting to write it to message");
  }
  message.data = state.get_value();
}

template void write_message<std_msgs::msg::Float64, double>(
    std_msgs::msg::Float64& message, const Parameter<double>& state, const rclcpp::Time&
);

template void write_message<std_msgs::msg::Float64MultiArray, std::vector<double>>(
    std_msgs::msg::Float64MultiArray& message, const Parameter<std::vector<double>>& state, const rclcpp::Time&
);

template void write_message<std_msgs::msg::Bool, bool>(
    std_msgs::msg::Bool& message, const Parameter<bool>& state, const rclcpp::Time&
);

template void write_message<std_msgs::msg::String, std::string>(
    std_msgs::msg::String& message, const Parameter<std::string>& state, const rclcpp::Time&
);

template<>
void
write_message(geometry_msgs::msg::Transform& message, const Parameter<CartesianPose>& state, const rclcpp::Time& time) {
  write_message(message, state.get_value(), time);
}

template<>
void write_message(
    geometry_msgs::msg::TransformStamped& message, const Parameter<CartesianPose>& state, const rclcpp::Time& time
) {
  write_message(message, state.get_value(), time);
}

template<>
void write_message(tf2_msgs::msg::TFMessage& message, const Parameter<CartesianPose>& state, const rclcpp::Time& time) {
  write_message(message, state.get_value(), time);
}

void write_message(std_msgs::msg::Bool& message, const bool& state, const rclcpp::Time&) {
  message.data = state;
}

void write_message(std_msgs::msg::Float64& message, const double& state, const rclcpp::Time&) {
  message.data = state;
}

void write_message(std_msgs::msg::Float64MultiArray& message, const std::vector<double>& state, const rclcpp::Time&) {
  message.data = state;
}

void write_message(std_msgs::msg::Int32& message, const int& state, const rclcpp::Time&) {
  message.data = state;
}

void write_message(std_msgs::msg::String& message, const std::string& state, const rclcpp::Time&) {
  message.data = state;
}
}// namespace modulo_core::translators
