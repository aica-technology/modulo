class ComponentError(Exception):
    """
    A base class for all component exceptions.
    """
    def __init__(self, message: str):
        super().__init__(message)


class ComponentParameterError(ComponentError):
    """
    An exception class to notify errors with component parameters. This is an exception class to be thrown if there is a
    problem with component parameters (overriding, inconsistent types, undeclared, ...).
    """
    def __init__(self, message: str):
        super().__init__(message)


class LookupTransformError(ComponentError):
    """
    An exception class to notify an error while looking up TF transforms. This is an exception class to be thrown if
    there is a problem with looking up a TF transform (unconfigured buffer/listener, TF2 exception).
    """
    def __init__(self, message: str):
        super().__init__(message)