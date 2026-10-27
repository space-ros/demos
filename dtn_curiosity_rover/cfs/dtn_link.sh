#!/bin/bash
# Control this node's side of the DTN link (the interface on the 172.28.10.0/24 network).
#
#   dtn_link.sh delay <ms>   one-way light-time delay, link up
#   dtn_link.sh down         link outage: drop everything this node sends
#   dtn_link.sh up           end the outage, keep the current delay
#   dtn_link.sh status       show the current setting
#
# Only IP packets (the bundles and custody signals) are delayed or dropped. ARP is
# left alone: with a multi-second delay, address resolution would time out and drop
# the first bundle sent on the link.
set -euo pipefail

IFACE=$(ip -o -4 addr show | awk '$4 ~ /^172\.28\.10\./ {print $2; exit}')
STATE=/tmp/dtn_link_delay_ms

if [ -z "${IFACE}" ]; then
  echo "dtn_link: no interface on the DTN link network" >&2
  exit 1
fi

delay_ms() { cat "${STATE}" 2>/dev/null || echo 0; }

# Band 1 (default): no shaping, used by ARP. Band 2: netem, used by all IP packets.
apply() {
  tc qdisc replace dev "${IFACE}" root handle 1: prio bands 2 priomap 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
  tc qdisc replace dev "${IFACE}" parent 1:2 handle 20: netem delay "$(delay_ms)ms" "$@" limit 100000
  tc filter replace dev "${IFACE}" parent 1: protocol ip prio 1 u32 match u32 0 0 flowid 1:2
}

case "${1:-status}" in
  delay)
    echo "${2:?delay in ms}" > "${STATE}"
    apply
    echo "dtn_link: ${IFACE} up, one-way delay $(delay_ms) ms"
    ;;
  down)
    apply loss 100%
    echo "dtn_link: ${IFACE} DOWN (all bundles dropped)"
    ;;
  up)
    apply
    echo "dtn_link: ${IFACE} up, one-way delay $(delay_ms) ms"
    ;;
  status)
    tc qdisc show dev "${IFACE}" | grep netem
    ;;
  *)
    echo "usage: $0 delay <ms> | down | up | status" >&2
    exit 1
    ;;
esac
