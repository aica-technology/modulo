#pragma once

#include <variant>

namespace modulo_core {

/**
 * @typedef PredicateVariant.
 */
typedef std::variant<bool, std::function<bool(void)>> PredicateVariant;

}// namespace modulo_components::modulo_utils
