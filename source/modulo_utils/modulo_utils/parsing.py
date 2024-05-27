import re


def topic_validation_warning(name: str, type: str):
    """
    This function returns a default validation warning for topic names.

    :param name: The pre-validation signal name
    :param type: One of input|output
    :return: The validation warning
    """
    return f"The parsed signal name for {type} '{name}' is empty. Provide a string with valid characters for the signal name ([a-zA-Z0-9_])."


def parse_topic_name(topic_name: str) -> str:
    """
    Parse a string topic name from a user-provided input.
    This functions removes all characters different from a-z, A-Z, 0-9, and _ from a string and transforms
    uppercase letters into lowercase letters. Additionally, it removes all leading numbers and underscores, such that
    the resulting string starts with a letter a-z.

    :param topic_name: The input string
    :return: The sanitized string
    """
    sanitized_string = re.sub("\W", "", topic_name, flags=re.ASCII).lower()
    sanitized_string = sanitized_string.lstrip("0123456789_")
    return sanitized_string
