#pragma once

#include "modulo_core/EncodedState.hpp"

#include <state_representation/State.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

namespace modulo_core::concepts {

// Data type concepts

template<typename T>
concept PrimitiveDataT = std::same_as<T, bool> || std::same_as<T, double> || std::same_as<T, std::vector<double>>
    || std::same_as<T, int> || std::same_as<T, std::string>;

template<typename T>
concept CoreDataT = std::derived_from<T, state_representation::State> || PrimitiveDataT<T>;

// Message type concepts

template<typename T>
concept TranslatedMsgT = std::same_as<T, std_msgs::msg::Bool> || std::same_as<T, std_msgs::msg::Float64>
    || std::same_as<T, std_msgs::msg::Float64MultiArray> || std::same_as<T, std_msgs::msg::Int32>
    || std::same_as<T, std_msgs::msg::String> || std::same_as<T, modulo_core::EncodedState>;

template<typename T>
concept EncodedT = std::same_as<T, modulo_core::EncodedState>;

template<typename T>
concept CustomT = !CoreDataT<T> && !EncodedT<T>;

}// namespace modulo_core::concepts
