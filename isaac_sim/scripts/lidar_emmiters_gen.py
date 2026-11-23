"""
This script generates parameters for a lidar sensor, such as azimuthDeg, elevationDeg, 
and fireTimeNs. The number of emitters and the angular range are configurable, and the 
values are printed in a format suitable for JSON.
"""

import numpy as np

# Constants
NUM_EMITTERS = 640  # Number of emitters

# Generate 640 azimuth values ranging from 180 to -180 degrees
azimuth_deg = np.linspace(180, -180, NUM_EMITTERS).tolist()

# All elevationDeg values are set to 0
elevation_deg = [0] * NUM_EMITTERS

# All fireTimeNs values are set to 0
fire_time_ns = [0] * NUM_EMITTERS


def print_parameter(name, values, is_last=False):
    """
    Prints the provided parameter name and its values in the format:
    "name": [value1, value2, ..., valueN], or without a comma if it's the last parameter.

    Args:
        name (str): The name of the parameter.
        values (list): A list of values to be printed for the parameter.
        is_last (bool): If True, no comma will be added at the end.
    """
    formatted_values = ", ".join(map(str, values))  # Joining the list with commas
    if is_last:
        print(f'"{name}": [{formatted_values}]')  # No comma after the closing bracket
    else:
        print(f'"{name}": [{formatted_values}],')  # Add comma after the closing bracket

# Print azimuthDeg, elevationDeg with commas, and fireTimeNs without
print_parameter("azimuthDeg", azimuth_deg)
print_parameter("elevationDeg", elevation_deg)
print_parameter("fireTimeNs", fire_time_ns, is_last=True)
