class ModuloError(RuntimeError):
    """
    A base class for all modulo exceptions.
    """

    def __init__(self, message: str, prefix="ModuloError"):
        super().__init__(f"{prefix}: {message}")


class AddServiceError(ModuloError):
    """
    An exception class to notify errors when adding a service. This is an exception class to be thrown if there is a
    problem while adding a service to a modulo class.
    """

    def __init__(self, message: str):
        super().__init__(message, "AddServiceError")


class AddSignalError(ModuloError):
    """
    An exception class to notify errors when adding a signal. This is an exception class to be thrown if there is a
    problem while adding a signal to a modulo class.
    """

    def __init__(self, message: str):
        super().__init__(message, "AddSignalError")


class ParameterError(ModuloError):
    """
    An exception class to notify errors with parameters. This is an exception class to be thrown if there is a problem
    with parameters (overriding, inconsistent types, undeclared, ...).
    """

    def __init__(self, message: str):
        super().__init__(message, "ParameterError")


class LookupTransformError(ModuloError):
    """
    An exception class to notify an error while looking up TF transforms. This is an exception class to be thrown if
    there is a problem with looking up a TF transform (unconfigured buffer/listener, TF2 exception).
    """

    def __init__(self, message: str):
        super().__init__(message, "LookupTransformError")
