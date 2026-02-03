#!/bin/bash
set -e

# Setup the Demo environment
source "${DEMO_DIR}/install/setup.bash"
exec "$@"
