#!/bin/bash
set -e

# Source all relevant worksapces
source "/opt/ros/humble/setup.bash"
source "${SPACEROS_DIR}/install/setup.bash"
source "${MOVEIT2_DIR}/install/setup.bash"
source "${DEMO_DIR}/install/setup.bash"
exec "$@"
