#!/bin/bash
set -e

# Setup the Demo environment
source "${SPACEROS_DIR}/install/setup.bash"
source /opt/ros/humble/setup.bash
source "${DEMO_DIR}/install/setup.bash"
exec "$@"