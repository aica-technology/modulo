#pragma once

#include <stdexcept>
#include <string>

/**
 * @namespace modulo_core::exceptions
 * @brief Modulo Core exceptions module for defining exception classes.
 */
namespace modulo_core::exceptions {

/**
 * @class CoreException
 * @brief A base class for all core exceptions.
 * @details This inherits from std::runtime_exception.
 */
class CoreException : public std::runtime_error {
public:
  explicit CoreException(const std::string& msg) : CoreException("CoreException", msg){};

protected:
  CoreException(const std::string& prefix, const std::string& msg) : std::runtime_error(prefix + ": " + msg) {}
};

/**
 * @class AddServiceException
 * @brief An exception class to notify errors when adding a service.
 * @details This is an exception class to be thrown if there is a problem while adding a service to a modulo class.
 */
class AddServiceException : public CoreException {
public:
  explicit AddServiceException(const std::string& msg) : CoreException("AddServiceException", msg) {}
};

/**
 * @class AddSignalException
 * @brief An exception class to notify errors when adding a signal.
 * @details This is an exception class to be thrown if there is a problem while adding a signal to a modulo class.
 */
class AddSignalException : public CoreException {
public:
  explicit AddSignalException(const std::string& msg) : CoreException("AddSignalException", msg) {}
};

/**
 * @class InvalidPointerCastException
 * @brief An exception class to notify if the result of getting an instance of a derived class through dynamic
 * down-casting of an object of the base class is not a correctly typed instance of the derived class.
 */
class InvalidPointerCastException : public CoreException {
public:
  explicit InvalidPointerCastException(const std::string& msg) : CoreException("InvalidPointerCastException", msg) {}
};

/**
 * @class InvalidPointerException
 * @brief An exception class to notify if an object has no reference count (if the object is not owned by any pointer)
 * when attempting to get a derived instance through dynamic down-casting.
 */
class InvalidPointerException : public CoreException {
public:
  explicit InvalidPointerException(const std::string& msg) : CoreException("InvalidPointerException", msg) {}
};

/**
 * @class LookupTransformException
 * @brief An exception class to notify an error while looking up TF transforms.
 * @details This is an exception class to be thrown if there is a problem with looking up a TF transform
 * (unconfigured buffer/listener, TF2 exception).
 */
class LookupTransformException : public CoreException {
public:
  explicit LookupTransformException(const std::string& msg) : CoreException("LookupTransformException", msg) {}
};

/**
 * @class MessageTranslationException
 * @brief An exception class to notify that the translation of a ROS message failed.
 */
class MessageTranslationException : public CoreException {
public:
  explicit MessageTranslationException(const std::string& msg) : CoreException("MessageTranslationException", msg) {}
};

/**
 * @class NullPointerException
 * @brief An exception class to notify that a certain pointer is null.
 * @details This is an exception class to be thrown if a pointer is null or is trying to be set to a null pointer.
 */
class NullPointerException : public CoreException {
public:
  explicit NullPointerException(const std::string& msg) : CoreException("NullPointerException", msg) {}
};

/**
 * @class ParameterException
 * @brief An exception class to notify errors with parameters in modulo classes.
 * @details This is an exception class to be thrown if there is a problem with parameters
 * (overriding, inconsistent types, undeclared, ...).
 */
class ParameterException : public CoreException {
public:
  explicit ParameterException(const std::string& msg) : CoreException("ParameterException", msg) {}
};

/**
 * @class ParameterTranslationException
 * @brief An exception class to notify incompatibility when translating parameters from different sources.
 * @details This is an exception class to be thrown if there is a problem while translating from a ROS parameter to a
 * state_representation parameter and vice versa.
 */
class ParameterTranslationException : public CoreException {
public:
  explicit ParameterTranslationException(const std::string& msg)
      : CoreException("ParameterTranslationException", msg) {}
};
}// namespace modulo_core::exceptions
