import re
from typing import List

from rclpy import Parameter


def parse_topic_name(topic_name: str) -> str:
    """
    Parse a string topic name from a user-provided input.
    This functions removes all characters different from a-z, A-Z, 0-9, and _ from a string.

    :param topic_name: The input string
    :return: The sanitized string
    """
    sanitized_string = re.sub("\W", "", topic_name, flags=re.ASCII).lower()
    sanitized_string = sanitized_string.lstrip("_")
    return sanitized_string


def modify_parameter_overrides(parameter_overrides: List[Parameter]) -> List[Parameter]:
    """
    Modify parameter overrides to handle the rate and period parameters.
    This function checks for existence of the rate and period parameter in the parameter overrides and modifies them to
    correspond to the same value. If both the rate and the period parameter exist, the period will be set from the rate.

    :param parameter_overrides: The parameter overrides passed to the component constructor
    :return: The modified parameter overrides 
    """
    rate = None
    period = None
    parameters = []
    for parameter in parameter_overrides:
        if parameter.name == "rate":
            rate = parameter
        elif parameter.name == "period":
            period = parameter
        else:
            parameters.append(parameter)
    
    if rate is not None and period is not None:
        rate = Parameter("rate", value=rate.get_parameter_value().integer_value)
        period = Parameter("period", value=1.0 / rate.get_parameter_value().integer_value)
    elif period is not None:
        period = Parameter("period", value=period.get_parameter_value().double_value)
        rate = Parameter("rate", value=int(1.0 / period.get_parameter_value().double_value))
    else:
        return parameters

    parameters.append(rate)
    parameters.append(period)
    return parameters
