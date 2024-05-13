#pragma once

#include <variant>

namespace modulo_utils {

typedef std::variant<bool, std::function<bool(void)>> PredicateVariant;

}// namespace modulo_components::modulo_utils
