#include "modulo_core/translators/message_readers.hpp"
#include <rclcpp/time.hpp>
#include <stdexcept>

namespace modulo_core::translators {

static Eigen::Vector3d read_point(const geometry_msgs::msg::Point& message) {
  return {message.x, message.y, message.z};
}

static Eigen::Vector3d read_vector3(const geometry_msgs::msg::Vector3& message) {
  return {message.x, message.y, message.z};
}

static Eigen::Quaterniond read_quaternion(const geometry_msgs::msg::Quaternion& message) {
  return {message.w, message.x, message.y, message.z};
}

void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Accel& message) {
  state.set_linear_acceleration(read_vector3(message.linear));
  state.set_angular_acceleration(read_vector3(message.angular));
}

void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::AccelStamped& message) {
  state.set_reference_frame(message.header.frame_id);
  read_message(state, message.accel);
}

void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Pose& message) {
  state.set_position(read_point(message.position));
  state.set_orientation(read_quaternion(message.orientation));
}

void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::PoseStamped& message) {
  state.set_reference_frame(message.header.frame_id);
  read_message(state, message.pose);
}

void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Transform& message) {
  state.set_position(read_vector3(message.translation));
  state.set_orientation(read_quaternion(message.rotation));
}

void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::TransformStamped& message) {
  state.set_reference_frame(message.header.frame_id);
  state.set_name(message.child_frame_id);
  read_message(state, message.transform);
}

void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Twist& message) {
  state.set_linear_velocity(read_vector3(message.linear));
  state.set_angular_velocity(read_vector3(message.angular));
}

void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::TwistStamped& message) {
  state.set_reference_frame(message.header.frame_id);
  read_message(state, message.twist);
}

void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Wrench& message) {
  state.set_force(read_vector3(message.force));
  state.set_torque(read_vector3(message.torque));
}

void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::WrenchStamped& message) {
  state.set_reference_frame(message.header.frame_id);
  read_message(state, message.wrench);
}

void read_message(state_representation::JointState& state, const sensor_msgs::msg::JointState& message) {
  try {
    state.set_names(message.name);
    if (!message.position.empty()) {
      state.set_positions(Eigen::VectorXd::Map(message.position.data(), message.position.size()));
    }
    if (!message.velocity.empty()) {
      state.set_velocities(Eigen::VectorXd::Map(message.velocity.data(), message.velocity.size()));
    }
    if (!message.effort.empty()) {
      state.set_torques(Eigen::VectorXd::Map(message.effort.data(), message.effort.size()));
    }
  } catch (const std::exception& ex) {
    throw exceptions::MessageTranslationException(ex.what());
  }
}

void read_message(state_representation::JointTrajectory& state, const trajectory_msgs::msg::JointTrajectory& message) {
  try {
    state.set_name(message.header.frame_id);
    if (!message.joint_names.empty()) {
      state.set_joint_names(message.joint_names);
    } else {
      throw std::runtime_error("JointTrajectory message has no joint names");
    }
    std::chrono::nanoseconds time_from_start(0);
    for (unsigned int i = 0; i < message.points.size(); ++i) {
      state_representation::JointState point("point_" + std::to_string(i), state.get_joint_names());
      if (!message.points[i].positions.empty()) {
        point.set_positions(message.points[i].positions);
      }
      if (!message.points[i].velocities.empty()) {
        point.set_velocities(message.points[i].velocities);
      }
      if (!message.points[i].accelerations.empty()) {
        point.set_accelerations(message.points[i].accelerations);
      }
      if (!message.points[i].effort.empty()) {
        point.set_torques(message.points[i].effort);
      }
      auto ros_time_from_start = std::chrono::nanoseconds(message.points[i].time_from_start.nanosec);
      auto duration = ros_time_from_start - time_from_start;
      time_from_start = ros_time_from_start;
      state.add_point(point, duration);
    }
  } catch (const std::exception& ex) {
    throw exceptions::MessageTranslationException(ex.what());
  } catch (...) {
    throw exceptions::MessageTranslationException("Unknown error while reading JointTrajectory message");
  }
}

void read_message(bool& state, const std_msgs::msg::Bool& message) {
  state = message.data;
}

void read_message(double& state, const std_msgs::msg::Float64& message) {
  state = message.data;
}

void read_message(std::vector<double>& state, const std_msgs::msg::Float64MultiArray& message) {
  state = message.data;
}

void read_message(int& state, const std_msgs::msg::Int32& message) {
  state = message.data;
}

void read_message(std::string& state, const std_msgs::msg::String& message) {
  state = message.data;
}
}// namespace modulo_core::translators
