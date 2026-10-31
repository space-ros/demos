#!/bin/bash
# Automated checks for the DTN demo (run ./build.sh first).
#
#   ./test.sh         DTN checks on the two cFS nodes: delay, ordering, outage recovery
#   ./test.sh --sim   also drive the rover in a headless Gazebo and check that it moves
#
# Exits non-zero on the first failed check.
set -euo pipefail
export MSYS_NO_PATHCONV=1 # Git Bash on Windows: keep container paths as they are
cd "$(dirname "$0")"

DELAY_MS=3000
SIM=false
[ "${1:-}" = "--sim" ] && SIM=true

export LINK_DELAY_MS=${DELAY_MS}
COMPOSE=(docker compose)
OVERRIDE=""

pass() { echo "PASS: $*"; }
fail() { echo "FAIL: $*" >&2; exit 1; }
now_ms() { date +%s%3N; }

cleanup() {
  "${COMPOSE[@]}" down -t 2 > /dev/null 2>&1 || true
  if [ -n "${OVERRIDE}" ]; then rm -f "${OVERRIDE}"; fi
}
trap cleanup EXIT
trap 'echo "FAIL: command failed (line ${LINENO}): ${BASH_COMMAND}" >&2' ERR

if ${SIM}; then
  # Gazebo server only (-s), so the check runs without a display
  OVERRIDE=.test-sim-override.yml
  cat > "${OVERRIDE}" << 'EOF'
services:
  curiosity_gui:
    command:
      - bash
      - -c
      - >-
        F=$$(find ~/curiosity_ws/install -name curiosity_gazebo.launch.py | head -1) &&
        sed -i "s/' -r'/' -r -s'/" "$$F" &&
        source /opt/ros/jazzy/setup.bash && source ~/curiosity_ws/install/setup.bash &&
        ros2 launch curiosity_gazebo curiosity_gazebo.launch.py
EOF
  COMPOSE+=(-f docker-compose.yml -f "${OVERRIDE}")
fi

send() { docker exec dtn_ground sh -c "echo $1 > /tmp/ground_keys"; }
rover_lines() { docker logs dtn_rover_cfs 2>&1 | grep '^\[rover' || true; }
rover_count() { rover_lines | wc -l; }
ground_hash() { docker logs dtn_ground 2>&1 | grep '^\[ground' | tail -1 | sed -E 's/.*body #([0-9a-f]+).*/\1/'; }
rover_hashes() { rover_lines | sed -E 's/.*body #([0-9a-f]+).*/\1/'; }

# Wait until the rover has logged at least $1 commands, for at most $2 seconds
wait_rover() {
  local want=$1 deadline=$(( $(date +%s) + $2 ))
  while [ "$(rover_count)" -lt "${want}" ]; do
    [ "$(date +%s)" -ge "${deadline}" ] && return 1
    sleep 0.2
  done
}

wait_log() {
  local container=$1 pattern=$2 secs=$3
  timeout "${secs}" bash -c "until docker logs ${container} 2>&1 | grep -q '${pattern}'; do sleep 1; done" \
    || fail "${container} did not log '${pattern}' within ${secs} s"
}

"${COMPOSE[@]}" down -t 2 > /dev/null 2>&1 || true
if ${SIM}; then
  "${COMPOSE[@]}" up -d > /dev/null
else
  "${COMPOSE[@]}" up -d ground rover_cfs > /dev/null
fi
wait_log dtn_ground "contact-start" 60
wait_log dtn_rover_cfs "contact-start" 60
pass "both DTN nodes up, contact 0 started (one-way delay ${DELAY_MS} ms)"

# 1. Light-time delay
start=$(now_ms)
send w
hash=$(ground_hash)
wait_rover 1 30 || fail "command not delivered within 30 s"
elapsed=$(( $(now_ms) - start ))
[ "$(rover_hashes | tail -1)" = "${hash}" ] || fail "rover received a different command than was sent"
[ "${elapsed}" -ge "${DELAY_MS}" ] || fail "command arrived after ${elapsed} ms, before the ${DELAY_MS} ms light time"
[ "${elapsed}" -le $(( DELAY_MS + 5000 )) ] || fail "command arrived after ${elapsed} ms, expected about ${DELAY_MS} ms"
pass "command delivered after ${elapsed} ms (light time ${DELAY_MS} ms)"

# 2. Ordering: three different commands, one second apart
sent=()
for key in a d s; do
  send "${key}"
  sleep 0.3
  sent+=("$(ground_hash)")
  sleep 0.7
done
wait_rover 4 30 || fail "not all of three commands were delivered"
mapfile -t got < <(rover_hashes | tail -3)
[ "${got[*]}" = "${sent[*]}" ] || fail "commands arrived out of order: sent ${sent[*]}, got ${got[*]}"
pass "three commands delivered in the order sent"

# 3. Outage: a command sent while the link is down arrives after the link returns
./link.sh down > /dev/null
send x
hash=$(ground_hash)
sleep 8
[ "$(rover_count)" -eq 4 ] || fail "a command crossed the link during the outage"
./link.sh up > /dev/null
up=$(now_ms)
wait_rover 5 45 || fail "command sent during the outage was not delivered after the link came back"
[ "$(rover_hashes | tail -1)" = "${hash}" ] || fail "rover received a different command after the outage"
pass "command sent during the outage delivered $(( ($(now_ms) - up) / 1000 )) s after the link came back"

if ${SIM}; then
  # 4. The rover drives in Gazebo
  wait_log dtn_rover_cfs "connected to ROS 2 bridge" 120
  wait_log dtn_curiosity_gui "wheel_tree_position_controller into state active" 240
  pos_x() {
    docker exec dtn_rover_ros bash -c 'source /home/spaceros-user/curiosity_ws/install/setup.bash &&
      timeout 10 ros2 topic echo --once --field pose.pose.position.x /model/curiosity_mars_rover/odometry' \
      | head -1
  }
  x0=$(pos_x)
  send w
  sleep $(( DELAY_MS / 1000 + 12 ))
  x1=$(pos_x)
  awk -v a="${x0}" -v b="${x1}" 'BEGIN { exit !(b - a > 0.3) }' \
    || fail "rover did not drive forward (x ${x0} -> ${x1})"
  pass "rover drove forward in Gazebo (x ${x0} -> ${x1} m)"

  send x
  sleep $(( DELAY_MS / 1000 + 6 ))
  x2=$(pos_x)
  sleep 3
  x3=$(pos_x)
  awk -v a="${x2}" -v b="${x3}" 'BEGIN { d = b - a; if (d < 0) d = -d; exit !(d < 0.05) }' \
    || fail "rover did not stop (x ${x2} -> ${x3})"
  pass "rover stopped (x ${x3} m)"
fi

echo "All checks passed."
