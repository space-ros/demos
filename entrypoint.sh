#!/bin/bash
set -e

# Setup the Demo environment
# source "${DEMO_DIR}/install/setup.bash"
source "/home/spaceros-user/demo_ws/install/setup.bash"
exec "$@"
