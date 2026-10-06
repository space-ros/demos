#!/usr/bin/env bash

# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

show_help() {
    cat <<'EOF'
Usage: ./run.sh [OPTIONS]

Start the Basilisk Space ROS demo.

Options:
    -h, --help     Show this help message and exit.
    --no-vizard    Run the demo without Vizard visualization.
    --no-rviz      Run the demo without RViz visualization.
EOF
}

start_vizard=true
start_rviz=true
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            show_help
            exit 0
            ;;
        --no-vizard)
            start_vizard=false
            shift
            ;;
        --no-rviz)
            start_rviz=false
            shift
            ;;
        *)
            echo "Unknown option: $1" >&2
            show_help >&2
            exit 2
            ;;
    esac
done

rviz_mode=setpoint
if [[ "${start_rviz}" == false ]]; then
    rviz_mode=off
fi

export XAUTHORITY="${XAUTHORITY:-${HOME}/.Xauthority}"

video_gid="$(getent group video | cut -d: -f3 || true)"
render_gid="$(getent group render | cut -d: -f3 || true)"
export VIDEO_GID="${VIDEO_GID:-${video_gid:-44}}"
export RENDER_GID="${RENDER_GID:-${render_gid:-110}}"

if [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY must be set for RViz and Vizard." >&2
    exit 1
fi

if [[ ! -r "${XAUTHORITY}" ]]; then
    echo "XAUTHORITY does not point to a readable file: ${XAUTHORITY}" >&2
    exit 1
fi

xhost +local:docker >/dev/null

docker compose down --remove-orphans
docker compose up -d

source_setup='source /opt/ros/jazzy/setup.bash && source "${SPACEROS_DIR}/setup.bash" && source /opt/basilisk_ws/install/setup.bash'

# Wait for each component
wait_for_condition() {
    local description="$1"
    local probe="$2"
    local status="${3:-Ready}"
    local timeout_seconds=60
    local deadline=$((SECONDS + timeout_seconds))

    while (( SECONDS < deadline )); do
        if docker compose exec -T basilisk bash -lc "${source_setup} && ${probe}" >/dev/null 2>&1; then
            echo "[${description}] ${status}"
            return 0
        fi
        sleep 1
    done

    echo "Timed out waiting for ${description}." >&2
    echo "Check the component logs with: docker compose exec basilisk bash" >&2
    return 1
}

docker compose exec -T -d basilisk bash -lc "${source_setup} && ros2 launch bsk-ros2-bridge bridge.launch.py > /tmp/bridge.log 2>&1"
wait_for_condition "Basilisk-ROS 2 Bridge" "ros2 node list | grep -Eiq 'bridge'"

docker compose exec -T -d basilisk bash -lc "${source_setup} && ros2 launch bsk-ros2-mpc multi_mpc.launch.py agents:=\"bskSat0 bskSat1 bskSat2\" rviz_mode:=${rviz_mode} > /tmp/mpc.log 2>&1"
wait_for_condition "Basilisk MPCs" "ros2 node list | grep -Fxq '/bskSat0/bsk_mpc' && ros2 node list | grep -Fxq '/bskSat1/bsk_mpc' && ros2 node list | grep -Fxq '/bskSat2/bsk_mpc'"
if [[ "${start_rviz}" == true ]]; then
    wait_for_condition "RViz Service" "ros2 service list | grep -Fxq '/bskSat0/set_pose' && ros2 service list | grep -Fxq '/bskSat1/set_pose' && ros2 service list | grep -Fxq '/bskSat2/set_pose'"
fi

if [[ "${start_vizard}" == true ]]; then
    docker compose exec -T -d basilisk bash -lc "${source_setup} && python3 /opt/basilisk_astrodynamics/scenarioRosMultiAgentOrbit.py --stream-mode live > /tmp/basilisk.log 2>&1"
    echo "[Vizard] Starting DirectComm at tcp://localhost:5556"
    docker compose exec -T -d basilisk bash -lc "cd /opt/vizard && /usr/local/bin/Vizard.x86_64 --args -directComm tcp://localhost:5556 > /tmp/vizard.log 2>&1"
else
    docker compose exec -T -d basilisk bash -lc "${source_setup} && python3 /opt/basilisk_astrodynamics/scenarioRosMultiAgentOrbit.py --stream-mode broadcast > /tmp/basilisk.log 2>&1"
    echo "[Vizard] Disabled"
    echo "[Basilisk] Broadcast stream for Vizard available at tcp://localhost:5570"
fi

wait_for_condition "Basilisk MPCs" "ros2 topic list | grep -Fxq '/bskSat0/bsk/out/sc_states' && ros2 topic list | grep -Fxq '/bskSat1/bsk/out/sc_states' && ros2 topic list | grep -Fxq '/bskSat2/bsk/out/sc_states'" "State topics received"

echo "Basilisk Space ROS demo is running."
echo "Attach to the container with: docker compose exec basilisk bash"
if [[ "${start_rviz}" == true ]]; then
    echo "Use RViz interactive markers to set position and attitude targets for agents."
else
    echo "[RViz] Disabled"
fi
echo "Without RViz, publish PoseStamped setpoints to /<agent>/bsk_mpc/setpoint_pose."
