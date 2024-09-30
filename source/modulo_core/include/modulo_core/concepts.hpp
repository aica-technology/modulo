#pragma once

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "modulo_core/EncodedState.hpp"

#include <state_representation/State.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/joint/JointState.hpp>

namespace modulo_core::concepts {

// Data type concepts

template<typename T>
concept CorePrimitiveT = std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>
    || std::same_as<T, bool> || std::same_as<T, std::string> || std::same_as<T, std::vector<double>>;

template<typename T>
concept CoreT = std::derived_from<T, state_representation::State> || CorePrimitiveT<T>;

template<typename T>
concept CustomT = !CoreT<T>;

// Translation concepts

template<typename T>
concept TranslatedT = std::same_as<T, geometry_msgs::msg::Accel> || std::same_as<T, geometry_msgs::msg::AccelStamped>
    || std::same_as<T, geometry_msgs::msg::Pose> || std::same_as<T, geometry_msgs::msg::PoseStamped>
    || std::same_as<T, geometry_msgs::msg::Transform> || std::same_as<T, geometry_msgs::msg::TransformStamped>
    || std::same_as<T, geometry_msgs::msg::Twist> || std::same_as<T, geometry_msgs::msg::TwistStamped>
    || std::same_as<T, geometry_msgs::msg::Wrench> || std::same_as<T, geometry_msgs::msg::WrenchStamped>
    || std::same_as<T, sensor_msgs::msg::JointState> || std::same_as<T, tf2_msgs::msg::TFMessage>
    || std::same_as<T, std_msgs::msg::Bool> || std::same_as<T, std_msgs::msg::Float64>
    || std::same_as<T, std_msgs::msg::Float64MultiArray> || std::same_as<T, std_msgs::msg::Int32>
    || std::same_as<T, std_msgs::msg::String>;
}// namespace modulo_core::concepts
