#!/bin/bash
# Send one drive command from the ground node: ./drive.sh w|s|a|d|x
set -euo pipefail
export MSYS_NO_PATHCONV=1 # Git Bash on Windows: keep container paths as they are
case "${1:-}" in
  w|s|a|d|x) docker exec dtn_ground sh -c "echo $1 > /tmp/ground_keys" ;;
  *) echo "usage: $0 w|s|a|d|x" >&2; exit 1 ;;
esac
