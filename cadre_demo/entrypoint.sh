#!/usr/bin/env bash

# Source ROS 2 setup files
source /opt/ros/humble/setup.bash
source /root/cadre_demo/install/setup.bash

# Print debug information
echo "ROS 2 environment set up."
echo "Current working directory: $(pwd)"
echo "Contents of /root/cadre_demo/src:"
ls -l /root/cadre_demo/src

# Execute the command passed to the container
exec "$@"
