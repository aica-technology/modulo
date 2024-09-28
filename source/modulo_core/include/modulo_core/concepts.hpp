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

template<typename DataT>
concept StateRepresentationDataT = std::derived_from<DataT, state_representation::State>;

template<typename DataT>
concept CorePrimitivesT = std::same_as<DataT, int> || std::same_as<DataT, float> || std::same_as<DataT, double>
    || std::same_as<DataT, bool> || std::same_as<DataT, std::string> || std::same_as<DataT, std::vector<double>>;

template<typename DataT>
concept CoreDataT = StateRepresentationDataT<DataT> || CorePrimitivesT<DataT>;

template<typename DataT>
concept CustomDataT = !CoreDataT<DataT>;

// Translation concepts

template<typename DataT>
concept TranslatedDataT = std::same_as<DataT, geometry_msgs::msg::Accel>
    || std::same_as<DataT, geometry_msgs::msg::AccelStamped> || std::same_as<DataT, geometry_msgs::msg::Pose>
    || std::same_as<DataT, geometry_msgs::msg::PoseStamped> || std::same_as<DataT, geometry_msgs::msg::Transform>
    || std::same_as<DataT, geometry_msgs::msg::TransformStamped> || std::same_as<DataT, geometry_msgs::msg::Twist>
    || std::same_as<DataT, geometry_msgs::msg::TwistStamped> || std::same_as<DataT, geometry_msgs::msg::Wrench>
    || std::same_as<DataT, geometry_msgs::msg::WrenchStamped> || std::same_as<DataT, sensor_msgs::msg::JointState>
    || std::same_as<DataT, tf2_msgs::msg::TFMessage> || std::same_as<DataT, std_msgs::msg::Bool>
    || std::same_as<DataT, std_msgs::msg::Float64> || std::same_as<DataT, std_msgs::msg::Float64MultiArray>
    || std::same_as<DataT, std_msgs::msg::Int32> || std::same_as<DataT, std_msgs::msg::String>;

template<typename DataT>
concept TranslatedOrEncodedDataT = TranslatedDataT<DataT> || std::same_as<DataT, EncodedState>;

template<typename DataT>
concept NonTranslatedDataT = !(TranslatedDataT<DataT> || std::same_as<DataT, EncodedState>);

}// namespace modulo_core::concepts
