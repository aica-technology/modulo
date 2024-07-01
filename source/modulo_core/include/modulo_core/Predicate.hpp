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
  explicit Predicate(const std::function<bool(void)>& predicate_function)
      : predicate_(std::move(predicate_function)) {
    previous_value_ = !predicate_();
  }

  bool get_value() const {
    return predicate_();
  }

  void set_predicate(const std::function<bool(void)>& predicate_function) {
    predicate_ = predicate_function;
  }

  std::optional<bool> query() {
    auto new_value = predicate_();
    if (new_value != previous_value_) {
      previous_value_ = new_value;
      return new_value;
    }
    return {};
  }

private:
  std::function<bool(void)> predicate_;
  bool previous_value_;
};

}// namespace modulo_core
