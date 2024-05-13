#pragma once

#include <stdexcept>
#include <string>

/**
 * @namespace modulo_utils::exceptions
 * @brief Modulo exception classes.
 */
namespace modulo_utils::exceptions {

/**
 * @class ModuloException
 * @brief A base class for all modulo exceptions.
 * @details This inherits from std::runtime_exception.
 */
class ModuloException : public std::runtime_error {
public:
  explicit ModuloException(const std::string& msg) : ModuloException("ModuloException", msg){};

protected:
  ModuloException(const std::string& prefix, const std::string& msg) : std::runtime_error(prefix + ": " + msg) {}
};
}// namespace modulo_utils::exceptions
