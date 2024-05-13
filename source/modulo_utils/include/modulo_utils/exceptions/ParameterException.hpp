#pragma once

#include "modulo_utils/exceptions/ModuloException.hpp"

namespace modulo_utils::exceptions {

/**
 * @class ParameterException
 * @brief An exception class to notify errors with parameters.
 * @details This is an exception class to be thrown if there is a problem with parameters
 * (overriding, inconsistent types, undeclared, ...).
 */
class ParameterException : public ModuloException {
public:
  explicit ParameterException(const std::string& msg) : ModuloException("ParameterException", msg) {}
};
}// namespace modulo_utils::exceptions
