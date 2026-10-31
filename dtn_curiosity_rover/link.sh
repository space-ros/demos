#!/bin/bash
# Change the DTN link between the ground and rover nodes, on both ends.
#
#   ./link.sh delay <ms>   one-way light-time delay (e.g. 1300 for the Moon)
#   ./link.sh down         start a link outage
#   ./link.sh up           end the outage
#   ./link.sh status
set -euo pipefail
export MSYS_NO_PATHCONV=1 # Git Bash on Windows: keep container paths as they are
for node in dtn_ground dtn_rover_cfs; do
  echo -n "${node}: "
  docker exec "${node}" /cfs/dtn_link.sh "$@"
done
