#pragma once

#include "modulo_utils/exceptions/ModuloException.hpp"

namespace modulo_utils::exceptions {

/**
 * @class AddSignalException
 * @brief An exception class to notify errors when adding a signal.
 * @details This is an exception class to be thrown if there is a problem while adding a signal to a modulo class.
 */
class AddSignalException : public ModuloException {
public:
  explicit AddSignalException(const std::string& msg) : ModuloException("AddSignalException", msg) {}
};
}// namespace modulo_utils::exceptions
