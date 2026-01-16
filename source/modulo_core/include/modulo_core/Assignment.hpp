#pragma once

#include <functional>
#include <map>
#include <optional>
#include <string>

#include <state_representation/parameters/Parameter.hpp>

namespace modulo_core {

/**
 * @class Assignment
 */
class Assignment {
public:
  explicit Assignment(const state_representation::ParameterType& type) : type_(type) {}

  state_representation::ParameterType get_type() const { return type_; }

  template<typename T>
  void check_types(const T& value) {
    // Technically, the type can be inferred from the value. Is there a better way to extract
    // the type of the assignment to be made and check it against the defined one?
    auto parameter = state_representation::make_shared_parameter("", value);
    if (parameter->get_parameter_type() != type_) {
      //TODO: throw something more specific?
      throw std::runtime_error("Assignment type mismatch.");
    }
    //TODO: store last assigned value for future use: check to not reassign or similar
  }

private:
  state_representation::ParameterType type_;
};
}// namespace modulo_core
