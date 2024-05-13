#pragma once

#include "modulo_utils/exceptions/ModuloException.hpp"

namespace modulo_utils::exceptions {

/**
 * @class LookupTransformException
 * @brief An exception class to notify an error while looking up TF transforms.
 * @details This is an exception class to be thrown if there is a problem with looking up a TF transform
 * (unconfigured buffer/listener, TF2 exception).
 */
class LookupTransformException : public ModuloException {
public:
  explicit LookupTransformException(const std::string& msg) : ModuloException("LookupTransformException", msg) {}
};
}// namespace modulo_utils::exceptions
