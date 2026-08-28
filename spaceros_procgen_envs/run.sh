#!/usr/bin/env bash
### Run a container with the specified image and command
### Usage: run.sh [-v HOST_DIR:DOCKER_DIR:OPTIONS] [-e ENV=VALUE] [TAG] [CMD]
###   -v HOST_DIR:DOCKER_DIR:OPTIONS  Mount the HOST_DIR to the DOCKER_DIR with the specified OPTIONS
###   -e ENV=VALUE                    Set the environment variable ENV to VALUE
###   TAG                             The tag of the image to run (default: "openrobotics/spaceros_procgen_envs")
###   CMD                             The command to run in the container (default: default command in the Dockerfile)

set -eo pipefail

## Configuration
IMAGE_NAME="${IMAGE_NAME:-"openrobotics/spaceros_procgen_envs"}"
# Flags for running the container
DOCKER_RUN_OPTS="${DOCKER_RUN_OPTS:-
    --interactive
    --tty
    --rm
    --network host
    --ipc host
}"
# Flags for enabling GPU and GUI (X11) inside the container
WITH_GPU="${WITH_GPU:-true}"
WITH_GUI="${WITH_GUI:-true}"
# List of volumes to mount (can be updated by passing -v HOST_DIR:DOCKER_DIR:OPTIONS)
DEMO_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
CONTAINER_HOME="/home/spaceros-user"
CUSTOM_VOLUMES=(
    "/etc/localtime:/etc/localtime:ro"
    # Demo
    "${DEMO_DIR}:${CONTAINER_HOME}/spaceros_procgen_envs:rw"
    # Cache (compiled shaders, proceduraly generated assets, etc.)
    "${HOME}/.cache/spaceros_demos/isaac_sim/computecache:${CONTAINER_HOME}/.nv/ComputeCache:rw"
    "${HOME}/.cache/spaceros_demos/isaac_sim/glcache:${CONTAINER_HOME}/.cache/nvidia/GLCache:rw"
    "${HOME}/.cache/spaceros_demos/isaac_sim/kit:${CONTAINER_HOME}/isaac-sim/kit/cache:rw"
    "${HOME}/.cache/spaceros_demos/isaac_sim/ov:${CONTAINER_HOME}/.cache/ov:rw"
    "${HOME}/.cache/spaceros_demos/procgen_envs:${CONTAINER_HOME}/.cache/spaceros_procgen_envs:rw"
)
# List of environment variables to set (can be updated by passing -e ENV=VALUE)
CUSTOM_ENVS=(
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-"0"}"
    RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-"rmw_cyclonedds_cpp"}"
)

# Create cache directories with appropriate permissions for the user inside the container
for VOLUME in "${CUSTOM_VOLUMES[@]}"; do
    HOST_DIR="${VOLUME%%:*}"
    if [[ "${HOST_DIR}" == *cache* ]]; then
        mkdir -p "${HOST_DIR}"
        chmod 777 "${HOST_DIR}"
    fi
done
find "${DEMO_DIR}" -exec bash -c 'p=$(stat -c "%a" "$0"); op=${p:0:1}; chmod "${op}${op}${op}" "$0" 2>/dev/null' {} \; 2>/dev/null || :

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
    # If the container name is already in use, append a unique (incremental) numerical suffix
    if ${WITH_SUDO} docker container list --all --format "{{.Names}}" | grep -qi "${CONTAINER_NAME}"; then
        CONTAINER_NAME="${CONTAINER_NAME}1"
        while ${WITH_SUDO} docker container list --all --format "{{.Names}}" | grep -qi "${CONTAINER_NAME}"; do
            CONTAINER_NAME="${CONTAINER_NAME%?}$((${CONTAINER_NAME: -1} + 1))"
        done
    fi
fi
DOCKER_RUN_OPTS="--name ${CONTAINER_NAME} ${DOCKER_RUN_OPTS}"

## Parse volumes and environment variables
while getopts ":v:e:" opt; do
    case "${opt}" in
        v) CUSTOM_VOLUMES+=("${OPTARG}") ;;
        e) CUSTOM_ENVS+=("${OPTARG}") ;;
        *)
            echo >&2 "Usage: ${0} [-v HOST_DIR:DOCKER_DIR:OPTIONS] [-e ENV=VALUE] [TAG] [CMD]"
            exit 2
            ;;
    esac
done
shift "$((OPTIND - 1))"

## Parse TAG and CMD positional arguments
if [ "${#}" -gt "0" ]; then
    if [[ $(${WITH_SUDO} docker images --format "{{.Tag}}" "${IMAGE_NAME}") =~ (^|[[:space:]])${1}($|[[:space:]]) || $(curl -fsSL "https://registry.hub.docker.com/v2/repositories/${IMAGE_NAME}/tags" 2>/dev/null | grep -Poe '(?<=(\"name\":\")).*?(?=\")') =~ (^|[[:space:]])${1}($|[[:space:]]) ]]; then
        # Use the first argument as a tag is such tag exists either locally or on the remote registry
        IMAGE_NAME="${IMAGE_NAME}:${1}"
        CMD=${*:2}
    else
        CMD=${*:1}
    fi
fi

## GPU
if [[ "${WITH_GPU,,}" = true ]]; then
    check_nvidia_gpu() {
        if [[ -n "${WITH_GPU_FORCE_NVIDIA}" ]]; then
            if [[ "${WITH_GPU_FORCE_NVIDIA,,}" = true ]]; then
                echo "[INFO] NVIDIA GPU is force-enabled via \`WITH_GPU_FORCE_NVIDIA=true\`."
                return 0 # NVIDIA GPU is force-enabled
            else
                echo "[INFO] NVIDIA GPU is force-disabled via \`WITH_GPU_FORCE_NVIDIA=false\`."
                return 1 # NVIDIA GPU is force-disabled
            fi
        elif ! lshw -C display 2>/dev/null | grep -qi "vendor.*nvidia"; then
            return 1 # NVIDIA GPU is not present
        elif ! command -v nvidia-smi >/dev/null 2>&1; then
            echo >&2 -e "\e[33m[WARN] NVIDIA GPU is detected, but its functionality cannot be verified. This container will not be able to use the GPU. Please install nvidia-utils on the host system or force-enable NVIDIA GPU via \`WITH_GPU_FORCE_NVIDIA=true\` environment variable.\e[0m"
            return 1 # NVIDIA GPU is present but nvidia-utils not installed
        elif ! nvidia-smi -L &>/dev/null; then
            echo >&2 -e "\e[33m[WARN] NVIDIA GPU is detected, but it does not seem to be working properly. This container will not be able to use the GPU. Please ensure the NVIDIA drivers are properly installed on the host system.\e[0m"
            return 1 # NVIDIA GPU is present but is not working properly
        else
            return 0 # NVIDIA GPU is present and appears to be working
        fi
    }
    if check_nvidia_gpu; then
        # Enable GPU either via NVIDIA Container Toolkit or NVIDIA Docker (depending on Docker version)
        DOCKER_VERSION="$(${WITH_SUDO} docker version --format '{{.Server.Version}}')"
        MIN_VERSION_FOR_TOOLKIT="19.3"
        if [ "$(printf '%s\n' "${MIN_VERSION_FOR_TOOLKIT}" "${DOCKER_VERSION}" | sort -V | head -n1)" = "$MIN_VERSION_FOR_TOOLKIT" ]; then
            GPU_OPT="--gpus all"
        else
            GPU_OPT="--runtime nvidia"
        fi
        GPU_ENVS=(
            NVIDIA_VISIBLE_DEVICES="all"
            NVIDIA_DRIVER_CAPABILITIES="all"
        )
    fi
    if [[ -e /dev/dri ]]; then
        GPU_OPT="${GPU_OPT} --device=/dev/dri:/dev/dri"
        if [[ $(getent group video) ]]; then
            GPU_OPT="${GPU_OPT} --group-add video"
        fi
    fi
fi

## GUI
if [[ "${WITH_GUI,,}" = true ]]; then
    # To enable GUI, make sure processes in the container can connect to the x server
    XAUTH="${TMPDIR:-"/tmp"}/xauth_docker_${CONTAINER_NAME}"
    touch "${XAUTH}"
    chmod a+r "${XAUTH}"
    XAUTH_LIST=$(xauth nlist "${DISPLAY}")
    if [ -n "${XAUTH_LIST}" ]; then
        echo "${XAUTH_LIST}" | sed -e 's/^..../ffff/' | xauth -f "${XAUTH}" nmerge -
    fi
    # GUI-enabling volumes
    GUI_VOLUMES=(
        "${XAUTH}:${XAUTH}"
        "/tmp/.X11-unix:/tmp/.X11-unix"
        "/dev/input:/dev/input"
    )
    # GUI-enabling environment variables
    GUI_ENVS=(
        DISPLAY="${DISPLAY}"
        XAUTHORITY="${XAUTH}"
    )
fi

## Run the container
# shellcheck disable=SC2206
DOCKER_RUN_CMD=(
    ${WITH_SUDO} docker run
    "${DOCKER_RUN_OPTS}"
    "${GPU_OPT}"
    "${GPU_ENVS[@]/#/"--env "}"
    "${GUI_VOLUMES[@]/#/"--volume "}"
    "${GUI_ENVS[@]/#/"--env "}"
    "${CUSTOM_VOLUMES[@]/#/"--volume "}"
    "${CUSTOM_ENVS[@]/#/"--env "}"
    "${IMAGE_NAME}"
    "${CMD}"
)
echo -e "\033[1;90m[TRACE] ${DOCKER_RUN_CMD[*]}\033[0m" | xargs
# shellcheck disable=SC2048
exec ${DOCKER_RUN_CMD[*]}
