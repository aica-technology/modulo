import re


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
