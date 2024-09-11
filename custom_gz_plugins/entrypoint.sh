#!/bin/bash
set -e

# Setup the Demo environment
source "${SPACEROS_DIR}/install/setup.bash"
exec "$@"
