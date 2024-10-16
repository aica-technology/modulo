#pragma once

#include <functional>
#include <map>
#include <optional>
#include <string>

namespace modulo_core {

/**
 * @class Predicate
 */
class Predicate {
public:
  explicit Predicate(const std::function<bool(void)>& predicate_function) : predicate_(std::move(predicate_function)) {}

  bool get_value() const { return predicate_(); }

  void set_predicate(const std::function<bool(void)>& predicate_function) { predicate_ = predicate_function; }

  std::optional<bool> query() {
    if (const auto new_value = predicate_(); !previous_value_ || new_value != *previous_value_) {
      previous_value_ = new_value;
      return new_value;
    }
    return {};
  }

private:
  std::function<bool(void)> predicate_;
  std::optional<bool> previous_value_;
};

}// namespace modulo_core
