#pragma once

#include "modulo_utils/exceptions/ModuloException.hpp"

namespace modulo_utils::exceptions {

/**
 * @class AddServiceException
 * @brief An exception class to notify errors when adding a service.
 * @details This is an exception class to be thrown if there is a problem while adding a service to a modulo class.
 */
class AddServiceException : public ModuloException {
public:
  explicit AddServiceException(const std::string& msg) : ModuloException("AddServiceException", msg) {}
};
}// namespace modulo_utils::exceptions
