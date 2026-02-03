#!/usr/bin/env bash
set -euo pipefail

# Always start from a clean state (optional but nice)
docker compose down --remove-orphans

# Bring services up in foreground so logs are visible
# and Ctrl-C stops them.
docker compose up

# If you prefer auto-clean on exit, use a trap:
# trap 'docker compose down --remove-orphans' EXIT
# docker compose up
