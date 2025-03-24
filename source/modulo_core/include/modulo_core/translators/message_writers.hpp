#pragma once

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <clproto.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/joint/JointState.hpp>
#include <state_representation/trajectory/CartesianTrajectory.hpp>
#include <state_representation/trajectory/JointTrajectory.hpp>

#include "modulo_core/EncodedState.hpp"
#include "modulo_core/exceptions.hpp"

namespace modulo_core::translators {

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Accel
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    geometry_msgs::msg::Accel& message, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::AccelStamped
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    geometry_msgs::msg::AccelStamped& message, const state_representation::CartesianState& state,
    const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Pose
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    geometry_msgs::msg::Pose& message, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::PoseStamped
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    geometry_msgs::msg::PoseStamped& message, const state_representation::CartesianState& state,
    const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Transform
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    geometry_msgs::msg::Transform& message, const state_representation::CartesianState& state,
    const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::TransformStamped
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    geometry_msgs::msg::TransformStamped& message, const state_representation::CartesianState& state,
    const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Twist
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    geometry_msgs::msg::Twist& message, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::TwistStamped
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    geometry_msgs::msg::TwistStamped& message, const state_representation::CartesianState& state,
    const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::Wrench
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    geometry_msgs::msg::Wrench& message, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS geometry_msgs::msg::WrenchStamped
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    geometry_msgs::msg::WrenchStamped& message, const state_representation::CartesianState& state,
    const rclcpp::Time& time);

/**
 * @brief Convert a JointState to a ROS sensor_msgs::msg::JointState
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    sensor_msgs::msg::JointState& message, const state_representation::JointState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianState to a ROS tf2_msgs::msg::TFMessage
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    tf2_msgs::msg::TFMessage& message, const state_representation::CartesianState& state, const rclcpp::Time& time);

/**
 * @brief Convert a CartesianTrajectory to a ROS trajectory_msgs::msg::JointTrajectory
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
void write_message(
    trajectory_msgs::msg::JointTrajectory& message, const state_representation::JointTrajectory& state,
    const rclcpp::Time& time);

/**
 * @brief Convert a Parameter<T> to a ROS equivalent representation
 * @tparam T All types of parameters supported in ROS std messages
 * @tparam U All types of parameters supported in ROS std messages
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the provided state is empty.
 */
template<typename U, typename T>
void write_message(U& message, const state_representation::Parameter<T>& state, const rclcpp::Time&);

/**
 * @brief Convert a boolean to a ROS std_msgs::msg::Bool
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_message(std_msgs::msg::Bool& message, const bool& state, const rclcpp::Time& time);

/**
 * @brief Convert a double to a ROS std_msgs::msg::Float64
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_message(std_msgs::msg::Float64& message, const double& state, const rclcpp::Time& time);

/**
 * @brief Convert a vector of double to a ROS std_msgs::msg::Float64MultiArray
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_message(
    std_msgs::msg::Float64MultiArray& message, const std::vector<double>& state, const rclcpp::Time& time);

/**
 * @brief Convert an integer to a ROS std_msgs::msg::Int32
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_message(std_msgs::msg::Int32& message, const int& state, const rclcpp::Time& time);

/**
 * @brief Convert a string to a ROS std_msgs::msg::String
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 */
void write_message(std_msgs::msg::String& message, const std::string& state, const rclcpp::Time& time);

/**
 * @brief Convert a state to an EncodedState (std_msgs::msg::UInt8MultiArray) message using protobuf encoding
 * @tparam T A state_representation::State type
 * @param message The ROS message to populate
 * @param state The state to read from
 * @param time The time of the message
 * @throws modulo_core::exceptions::MessageTranslationException if the translation failed or is not supported.
 */
template<typename T>
inline void write_message(EncodedState& message, const T& state, const rclcpp::Time&) {
  try {
    std::string tmp = clproto::encode<T>(state);
    message.data = std::vector<unsigned char>(tmp.begin(), tmp.end());
  } catch (const std::exception& ex) {
    throw exceptions::MessageTranslationException(ex.what());
  }
}
}// namespace modulo_core::translators
