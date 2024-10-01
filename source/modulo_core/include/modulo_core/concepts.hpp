#pragma once

#include <state_representation/State.hpp>

namespace modulo_core::concepts {

// Data type concepts

template<typename T>
concept CorePrimitiveT = std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>
    || std::same_as<T, bool> || std::same_as<T, std::string> || std::same_as<T, std::vector<double>>;

template<typename T>
concept CoreT = std::derived_from<T, state_representation::State> || CorePrimitiveT<T>;

template<typename T>
concept CustomT = !CoreT<T>;

}// namespace modulo_core::concepts
