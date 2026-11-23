#!/bin/bash
set -e

# Underlay: apt RViz (installed in this image)
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Overlay: source-built Space ROS Nav2 workspace
source "/home/spaceros-user/nav2_ws/install/setup.bash"

# Overlay: demo RViz launch/config + source-built nav2_rviz_plugins
if [ -f "/home/spaceros-user/nav2_demo_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/home/spaceros-user/nav2_demo_ws/install/setup.bash"
fi

exec "$@"
