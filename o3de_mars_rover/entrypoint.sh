#!/bin/bash
set -e

# Source all relevant worksapces
source "${SPACEROS_DIR}/install/setup.bash"
source "${MOVEIT2_DIR}/install/setup.bash"
source "${DEMO_DIR}/install/setup.bash"
exec "$@"
