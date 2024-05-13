import re
from typing import List

from rclpy import Parameter


def modify_parameter_overrides(parameter_overrides: List[Parameter]) -> List[Parameter]:
    """
    Modify parameter overrides to handle the rate and period parameters.
    This function checks for existence of the rate and period parameter in the parameter overrides and modifies them to
    correspond to the same value. If both the rate and the period parameter exist, the period will be set from the rate.

    :param parameter_overrides: The parameter overrides passed to the component constructor
    :return: The modified parameter overrides 
    """
    rate = Parameter("rate", type_=Parameter.Type.NOT_SET)
    period = Parameter("period", type_=Parameter.Type.NOT_SET)
    parameters = []
    for parameter in parameter_overrides:
        if parameter.name == "rate":
            rate = parameter
        elif parameter.name == "period":
            period = parameter
        else:
            parameters.append(parameter)
    
    if rate.type_ != Parameter.Type.NOT_SET:
        period = Parameter("period", value=1.0 / rate.get_parameter_value().integer_value)
    elif period.type_ != Parameter.Type.NOT_SET:
        rate = Parameter("rate", value=int(1.0 / period.get_parameter_value().double_value))
    else:
        return parameters

    parameters.append(rate)
    parameters.append(period)
    return parameters
