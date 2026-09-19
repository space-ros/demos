## Base <https://hub.docker.com/r/osrf/space-ros>
ARG BASE_IMAGE_NAME="osrf/space-ros"
ARG BASE_IMAGE_TAG="latest"
FROM ${BASE_IMAGE_NAME}:${BASE_IMAGE_TAG}

## Use bash as the default shell
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

## Install system dependencies
# hadolint ignore=DL3004
RUN sudo apt-get update && \
    DEBIAN_FRONTEND=noninteractive sudo apt-get install -yq --no-install-recommends \
    # Common
    build-essential \
    ca-certificates \
    curl \
    git \
    mold \
    xz-utils \
    # Isaac Sim
    libgl1 \
    libglu1 \
    libxt-dev \
    # Blender
    libegl1 \
    # Video recording/processing
    ffmpeg && \
    sudo rm -rf /var/lib/apt/lists/*

## Install Rust
ARG RUST_VERSION="1.80"
RUN echo -e "\n# Rust ${RUST_VERSION}" >> /ros_entrypoint.sh && \
    echo "export PATH=\"${HOME}/.cargo/bin\${PATH:+:\${PATH}}\"" >> /ros_entrypoint.sh && \
    echo "export CARGO_TARGET_DIR=\"${HOME}/.cargo/target\"" >> /ros_entrypoint.sh && \
    echo "export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUSTFLAGS=\"-Clink-arg=-fuse-ld=mold -Ctarget-cpu=native\"" >> /ros_entrypoint.sh && \
    curl --proto "=https" --tlsv1.2 -sSfL "https://sh.rustup.rs" | sh -s -- --no-modify-path -y --default-toolchain "${RUST_VERSION}" --profile default

## Install Blender
ARG BLENDER_PATH="${HOME_DIR}/blender"
ARG BLENDER_VERSION="4.2.1"
RUN echo -e "\n# Blender ${BLENDER_VERSION}" >> /ros_entrypoint.sh && \
    echo "export PATH=\"${BLENDER_PATH}\${PATH:+:\${PATH}}\"" >> /ros_entrypoint.sh && \
    curl --proto "=https" --tlsv1.2 -sSfL "https://download.blender.org/release/Blender${BLENDER_VERSION%.*}/blender-${BLENDER_VERSION}-linux-x64.tar.xz" -o "/tmp/blender_${BLENDER_VERSION}.tar.xz" && \
    mkdir -p "${BLENDER_PATH}" && \
    tar xf "/tmp/blender_${BLENDER_VERSION}.tar.xz" -C "${BLENDER_PATH}" --strip-components=1 && \
    rm "/tmp/blender_${BLENDER_VERSION}.tar.xz"

## Install Isaac Lab
ARG ISAACLAB_PATH="${HOME_DIR}/isaaclab"
ARG ISAACLAB_REMOTE="https://github.com/isaac-sim/IsaacLab.git"
ARG ISAACLAB_BRANCH="main"
ARG ISAACLAB_COMMIT_SHA="af088f593216f7215310cf5e3a13a259d7320a63"
# hadolint ignore=SC2044
RUN echo -e "\n# Isaac Lab ${ISAACLAB_COMMIT_SHA}" >> /ros_entrypoint.sh && \
    echo "export ISAACLAB_PATH=\"${ISAACLAB_PATH}\"" >> /ros_entrypoint.sh && \
    git clone "${ISAACLAB_REMOTE}" "${ISAACLAB_PATH}" --branch "${ISAACLAB_BRANCH}" && \
    git -C "${ISAACLAB_PATH}" reset --hard "${ISAACLAB_COMMIT_SHA}" && \
    python3 -m pip install --no-cache-dir --upgrade pip==24.2 && \
    for extension in $(find -L "${ISAACLAB_PATH}/source/extensions" -mindepth 1 -maxdepth 1 -type d); do \
    if [ -f "${extension}/setup.py" ]; then \
    python3 -m pip install --no-cache-dir --editable "${extension}" ; \
    fi ; \
    done

## (Optional) Install ROS utilities to simplify the demonstration
ARG INSTALL_ROS_UTILS="true"
# hadolint ignore=SC1091,DL3004
RUN if [ "${INSTALL_ROS_UTILS}" = "true" ]; then \
    source /ros_entrypoint.sh -- && \
    echo -e "\n# ROS ${ROS_DISTRO} Utilities" >> /ros_entrypoint.sh && \
    echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\"" >> /ros_entrypoint.sh && \
    sudo apt-get update && \
    DEBIAN_FRONTEND=noninteractive sudo apt-get install -yq --no-install-recommends \
    "ros-${ROS_DISTRO}-rviz2" \
    "ros-${ROS_DISTRO}-teleop-twist-keyboard" \
    xterm && \
    sudo rm -rf /var/lib/apt/lists/* ; \
    fi

## Copy the source code into the image
ARG DEMO_PATH="${HOME_DIR}/spaceros_procgen_envs"
WORKDIR "${DEMO_PATH}"
COPY --chown=${USERNAME}:${USERNAME} . "${DEMO_PATH}"

## Install the demo as editable Python module
# hadolint ignore=SC1091
RUN source /ros_entrypoint.sh -- && \
    python3 -m pip install --no-cache-dir --editable "${DEMO_PATH}[all]"

## Install the demo as Space ROS package
ARG DEMO_SPACEROS_WS_PATH="${HOME_DIR}/spaceros/demo"
# hadolint ignore=SC1091
RUN mkdir -p "${DEMO_SPACEROS_WS_PATH}/src" && \
    ln -s "${DEMO_PATH}" "${DEMO_SPACEROS_WS_PATH}/src/spaceros_procgen_envs" && \
    source /ros_entrypoint.sh -- && \
    colcon build --merge-install --symlink-install --paths "${DEMO_PATH}" \
    --build-base "${DEMO_SPACEROS_WS_PATH}/build" \
    --install-base "${DEMO_SPACEROS_WS_PATH}/install" && \
    rm -rf ./log && \
    echo -e "\n# [Demo] Space ROS ProcGen Environments" >> /ros_entrypoint.sh && \
    echo "source \"${DEMO_SPACEROS_WS_PATH}/install/setup.bash\"" >> /ros_entrypoint.sh

## Skip writing Python bytecode to the disk
ENV PYTHONDONTWRITEBYTECODE=1

## Specify the path where Isaac Sim is expected to be mounted as a volume
ARG ISAAC_SIM_PATH="${HOME_DIR}/isaac-sim"
ENV ISAAC_SIM_PYTHON="${ISAAC_SIM_PATH}/python.sh"
ENV PYTHONEXE="/usr/bin/python3"
# hadolint ignore=SC2028
RUN echo -e "\n# Isaac Sim" >> /ros_entrypoint.sh && \
    echo "export ISAAC_SIM_PATH=\"${ISAAC_SIM_PATH}\"" >> /ros_entrypoint.sh && \
    echo "if [ -z \"\${SKIP_ISAAC_SIM_PATH_CHECK}\" ] && [ ! -d \"\${ISAAC_SIM_PATH}\" ]; then" >> /ros_entrypoint.sh && \
    echo "  echo -e \"\033[1;31m[Error] Isaac Sim must be mounted as a volume at \${ISAAC_SIM_PATH}\033[0m\"" >> /ros_entrypoint.sh && \
    echo "  exit 1" >> /ros_entrypoint.sh && \
    echo "fi" >> /ros_entrypoint.sh

## Finalize the entrypoint
# hadolint ignore=SC2016,DL3004
RUN sudo sed -i '/set -e/d' /ros_entrypoint.sh && \
    sudo sed -i '/exec "\$@"/d' /ros_entrypoint.sh && \
    echo -e "\n# Execute command" >> /ros_entrypoint.sh && \
    echo -en 'exec "${@}"\n' >> /ros_entrypoint.sh && \
    sed -i '$a source /ros_entrypoint.sh' ~/.bashrc

## Set the default command
CMD ["bash"]
