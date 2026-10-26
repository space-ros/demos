#!/usr/bin/env bash
### Join an active container with the specified name and command
### Usage: join.sh [CONTAINER_NAME_ID] [CMD]
###   CONTAINER_NAME_ID  The numerical suffix of the container name in case of multiple
###                      active containers with the same name (default: 0)
###   CMD                The command to run inside the container (default: "bash")

set -eo pipefail

## Configuration
IMAGE_NAME="${IMAGE_NAME:-"openrobotics/spaceros_procgen_envs"}"
# Flags for executing a command inside the container
DOCKER_EXEC_OPTS="${DOCKER_EXEC_OPTS:-
    --interactive
    --tty
}"
# Default command to execute inside the container
DEFAULT_CMD="bash"

## If the current user is not in the docker group, all docker commands will be run as root
WITH_SUDO=""
if ! grep -qi /etc/group -e "docker.*${USER}"; then
    echo "[INFO] The current user ${USER} is not detected in the docker group. All docker commands will be run as root."
    WITH_SUDO="sudo"
fi

## Use the provided container name or generate a unique one
if [ -z "${CONTAINER_NAME}" ]; then
    # Select the container name based on the image name
    CONTAINER_NAME="${IMAGE_NAME##*/}"
    # Replace all non-alphanumeric characters with underscores
    CONTAINER_NAME="${CONTAINER_NAME//[^a-zA-Z0-9]/_}"
fi

## Parse CMD if provided
if [ "${#}" -gt "0" ]; then
    # If the first argument is a positive integer, it is parsed as the suffix of the container name
    if [[ "${1}" =~ ^[0-9]+$ ]]; then
        CONTAINER_NAME_ID="${1}"
        if [ "${#}" -gt "1" ]; then
            CMD=${*:2}
        else
            CMD="${DEFAULT_CMD}"
        fi
    else
        CMD=${*:1}
    fi
else
    CMD="${DEFAULT_CMD}"
fi

## Verify that the container is active
RUNNING_CONTAINERS=$(${WITH_SUDO} docker container list --format "{{.Names}}" | grep -i "${CONTAINER_NAME}" || :)
RUNNING_CONTAINERS_COUNT=$(echo "${RUNNING_CONTAINERS}" | wc -w)
if [ "${RUNNING_CONTAINERS_COUNT}" -eq "0" ]; then
    echo >&2 -e "\033[1;31m[ERROR] There are no active containers with the name \"${CONTAINER_NAME}\". Start the container first before attempting to join it.\033[0m"
    exit 1
fi

print_running_containers_and_usage() {
    RUNNING_CONTAINERS=$(echo "${RUNNING_CONTAINERS}" | sort --version-sort)
    echo >&2 "Active *${CONTAINER_NAME}* containers:"
    i=0
    echo "${RUNNING_CONTAINERS}" | while read -r line; do
        echo >&2 -e "\t${i}: ${line}"
        i=$((i + 1))
    done
    echo >&2 "Usage: ${0} [CONTAINER_NAME_ID] [CMD]"
}
## If provided, append the numerical suffix to the container name
if [[ -n "${CONTAINER_NAME_ID}" ]]; then
    if [ "${CONTAINER_NAME_ID}" -eq "0" ]; then
        CONTAINER_NAME_ID=""
    fi
    # Make sure that the container with the specified suffix is active
    if ! echo "${RUNNING_CONTAINERS}" | grep -qi "${CONTAINER_NAME}${CONTAINER_NAME_ID}"; then
        echo >&2 -e "\033[1;31m[ERROR] Invalid argument \"${CONTAINER_NAME_ID}\" — there is no active container with the name \"${CONTAINER_NAME}${CONTAINER_NAME_ID}\".\033[0m"
        print_running_containers_and_usage
        exit 2
    fi
    CONTAINER_NAME="${CONTAINER_NAME}${CONTAINER_NAME_ID}"
else
    # Otherwise, check if there is only one active container with the specified name
    if [ "${RUNNING_CONTAINERS_COUNT}" -gt "1" ]; then
        echo >&2 -e "\033[1;31m[ERROR] More than one active *${CONTAINER_NAME}* container. Specify the suffix of the container name as the first argument.\033[0m"
        print_running_containers_and_usage
        exit 2
    else
        # If there is only one active container, use it regardless of the suffix
        CONTAINER_NAME="${RUNNING_CONTAINERS}"
    fi
fi

## Execute command inside the container
# shellcheck disable=SC2206
DOCKER_EXEC_CMD=(
    ${WITH_SUDO} docker exec
    "${DOCKER_EXEC_OPTS}"
    "${CONTAINER_NAME}"
    "${CMD}"
)
echo -e "\033[1;90m[TRACE] ${DOCKER_EXEC_CMD[*]}\033[0m" | xargs
# shellcheck disable=SC2048
exec ${DOCKER_EXEC_CMD[*]}
