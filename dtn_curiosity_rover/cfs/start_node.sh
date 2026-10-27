#!/bin/bash
# Start one DTN node of the demo: cFS with BPNode, then bring up its DTN contact.
#
#   start_node.sh ground   # cpu1, ipn:100, runs run_app (keyboard teleop)
#   start_node.sh rover    # cpu2, ipn:200, runs racs2_bridge_client
#
# LINK_DELAY_MS sets the one-way light-time delay this node adds to its DTN link.
set -euo pipefail

case "${1:-}" in
  ground) CPU=cpu1; CI_PORT=1234 ;;
  rover)  CPU=cpu2; CI_PORT=1235 ;;
  *) echo "usage: $0 ground|rover" >&2; exit 1 ;;
esac

/cfs/dtn_link.sh delay "${LINK_DELAY_MS:-5000}"

# BPNode keeps bundles in a SQLite store; start each run with an empty one
cd "/cfs/build/exe/${CPU}"
rm -f bplib-storage.db bplib-storage.db-shm bplib-storage.db-wal

# Once cFS accepts commands, set up and start contact 0 (the link to the other node).
# These are the BPNode ground commands CONTACT_SETUP (29) and CONTACT_START (30).
(
  until ss -lun | grep -q ":${CI_PORT} "; do sleep 0.5; done
  sleep 1
  for CC in 29 30; do
    /cfs/build/exe/host/cmdUtil --host=127.0.0.1 --port="${CI_PORT}" --endian=LE \
      --pktid=0x1838 --cmdcode="${CC}" --uint32=0 > /dev/null
  done
) &

exec "./core-${CPU}"
