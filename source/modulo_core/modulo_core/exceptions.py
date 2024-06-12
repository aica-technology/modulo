class CoreError(Exception):
    """
    A base class for all core exceptions.
    """

    def __init__(self, message: str, prefix="CoreError"):
        super().__init__(f"{prefix}: {message}")


class AddServiceError(CoreError):
    """
    An exception class to notify errors when adding a service. This is an exception class to be thrown if there is a
    problem while adding a service to a modulo class.
    """

    def __init__(self, message: str):
        super().__init__(message, "AddServiceError")


class AddSignalError(CoreError):
    """
    An exception class to notify errors when adding a signal. This is an exception class to be thrown if there is a
    problem while adding a signal to a modulo class.
    """

    def __init__(self, message: str):
        super().__init__(message, "AddSignalError")


class LookupTransformError(CoreError):
    """
    An exception class to notify an error while looking up TF transforms. This is an exception class to be thrown if
    there is a problem with looking up a TF transform (unconfigured buffer/listener, TF2 exception).
    """

    def __init__(self, message: str):
        super().__init__(message, "LookupTransformError")


class MessageTranslationError(CoreError):
    """
    An exception class to notify that the translation of a ROS message failed.
    """

    def __init__(self, message: str):
        super().__init__(message, "MessageTranslationError")


class ParameterError(CoreError):
    """
    An exception class to notify errors with parameters in modulo classes. This is an exception class to be thrown if
    there is a problem with parameters (overriding, inconsistent types, undeclared, ...).
    """

    def __init__(self, message: str):
        super().__init__(message, "ParameterError")


class ParameterTranslationError(CoreError):
    """
    An exception class to notify incompatibility when translating parameters from different sources. This is an
    exception class to be thrown if there is a problem while translating from a ROS parameter to a state_representation
    parameter and vice versa.
    """

    def __init__(self, message: str):
        super().__init__(message, "ParameterTranslationError")
