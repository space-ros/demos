import re

from omni.isaac.lab.utils.string import *  # noqa: F403

REGEX_CANONICALIZE_STR_PATTERN: re.Pattern = re.compile("[\W_]+")


def canonicalize_str(input: str) -> str:
    """
    Canonicalizes a string by converting it to lowercase and removing unwanted characters.

    This function processes the input string to ensure it is in a standardized format, making it suitable for consistent usage in applications. It utilizes a predefined regular expression pattern to eliminate any characters that do not meet the specified criteria.

    Args:
        input (str): The string to be canonicalized.

    Returns:
        str: The canonicalized version of the input string.
    """
    return REGEX_CANONICALIZE_STR_PATTERN.sub("", input.lower())


def sanitize_camera_name(name: str) -> str:
    for s in ["cam_", "camera_", "sensor_"]:
        name = name.replace(s, "")
    return name
