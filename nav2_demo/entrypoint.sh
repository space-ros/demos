#!/bin/bash
set -e

# Setup the Navigation2 environment
source "/home/spaceros-user/nav2_ws/install/setup.bash"

exec "$@"
