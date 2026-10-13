#!/usr/bin/env bash
### Build a Docker image with the specified tag
### Usage: build.sh [TAG] [BUILD_ARGS...]
###   TAG         The tag of the image to build (default: "openrobotics/spaceros_procgen_envs")
###   BUILD_ARGS  Additional arguments to forward to the `docker build` command
###               (e.g. --no-cache --build-arg KEY=VALUE)

set -eo pipefail

## Configuration
IMAGE_NAME="${IMAGE_NAME:-"openrobotics/spaceros_procgen_envs"}"

# Parse TAG and forward additional build arguments
if [ "${#}" -gt "0" ]; then
    if [[ "${1}" != "-"* ]]; then
        IMAGE_NAME="${IMAGE_NAME}:${1}"
        BUILD_ARGS=${*:2}
    else
        BUILD_ARGS=${*:1}
    fi
fi

echo "[INFO] Building Docker image '${IMAGE_NAME}'..."

# Localize build
DEMO_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
DOCKERFILE_PATH="${DEMO_DIR}/Dockerfile"

# If the current user is not in the docker group, all docker commands will be run as root
WITH_SUDO=""
if ! grep -qi /etc/group -e "docker.*${USER}"; then
    echo "INFO: The current user ${USER} is not detected in the docker group. All docker commands will be run as root."
    WITH_SUDO="sudo"
fi

# Check if Isaac Sim can be pulled to determine whether to bundle Isaac Sim or rely on a local installation
ISAAC_SIM_IMAGE_NAME="${ISAAC_SIM_IMAGE_NAME:-"nvcr.io/nvidia/isaac-sim"}"
ISAAC_SIM_IMAGE_TAG="${ISAAC_SIM_IMAGE_TAG:-"4.1.0"}"
if ${WITH_SUDO} docker pull "${ISAAC_SIM_IMAGE_NAME}:${ISAAC_SIM_IMAGE_TAG}" &>/dev/null; then
    DOCKERFILE_PATH+=".bundle_isaac_sim"
    BUILD_ARGS+=" --build-arg ISAAC_SIM_IMAGE_NAME=${ISAAC_SIM_IMAGE_NAME} --build-arg ISAAC_SIM_IMAGE_TAG=${ISAAC_SIM_IMAGE_TAG}"
else
    >&2 echo -e "\033[1;33m[WARN] This system is not logged into the NVIDIA NGC registry to pull '${ISAAC_SIM_IMAGE_NAME}:${ISAAC_SIM_IMAGE_TAG}' image"
    if [ -n "${CI}" ]; then
        >&2 echo -e "[WARN] Building Docker image without bundling Isaac Sim...\033[0m"
    else
        read -rp "[PROMPT] Do you want to continue with the build without Isaac Sim? (ADVANCED) [y/N] " CONTINUE
        if [[ ! "${CONTINUE}" =~ ^[Yy]$ ]]; then
            echo "[INFO] Exiting..."
            exit 0
        else
            >&2 echo -e "[WARN] Local installation of Isaac Sim will have to be mounted as a volume when running this demo using the '${IMAGE_NAME}' image\033[0m"
        fi
    fi
fi

# Build the image
DOCKER_BUILD_CMD=(
    "${WITH_SUDO}" docker build
    "${DEMO_DIR}"
    --tag "${IMAGE_NAME}"
    --file "${DOCKERFILE_PATH}"
    "${BUILD_ARGS}"
)
echo -e "\033[1;90m[TRACE] ${DOCKER_BUILD_CMD[*]}\033[0m"
# shellcheck disable=SC2048
exec ${DOCKER_BUILD_CMD[*]}
echo "[INFO] Docker image '${ORG}/${IMAGE}' built successfully"
