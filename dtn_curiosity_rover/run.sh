#!/bin/bash
# Start the demo and attach to the ground node's console to drive the rover.
# Detach without stopping the demo with Ctrl-P Ctrl-Q; stop it with: docker compose down
set -euo pipefail
cd "$(dirname "$0")"

docker compose down --remove-orphans
docker compose up -d

echo
echo "One-way light-time delay on the DTN link: ${LINK_DELAY_MS:-5000} ms"
echo "Drive with w (forward), s (backward), a (left), d (right), x (stop)."
echo "Change the link from another terminal: ./link.sh delay 1300 | down | up"
echo
docker attach dtn_ground
