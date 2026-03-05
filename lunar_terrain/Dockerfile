# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# A Docker configuration script to build the Space ROS image.
#
# The script provides the following build arguments:
#
#   VCS_REF     - The git revision of the Space ROS source code (no default value).
#   VERSION     - The version of Space ROS (default: "preview")

FROM osrf/space-ros:jazzy-2025.10.0

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Lunar Terrain"
LABEL org.label-schema.description="Lunar terrain demo on the Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Clone all space-ros sources
RUN sudo vcs import ${SPACEROS_DIR}/src < ${SPACEROS_DIR}/scripts/spaceros.repos

# Define a few key variables
ENV DEMO_DIR=${HOME}/demos_ws
ENV GZ_VERSION=harmonic
ENV GZ_PARTITION=spaceros
ENV GZ_SIM_RESOURCE_PATH=$DEMO_DIR/src/simulation/models/lunar_terrain/models
ENV ROS_DISTRO=jazzy

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Update the ROS package keys
ADD --chmod=644 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /usr/share/keyrings/ros-archive-keyring.gpg

# Make sure the latest versions of packages are installed
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update \
  && sudo apt-get dist-upgrade -y
RUN sudo rosdep init && rosdep update

# Install the various build and test tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt install -y \
  build-essential \
  clang-format \
  cmake \
  git \
  git-lfs \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosinstall-generator \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

# Install git-lfs for big files in simulation repo
RUN git lfs install

RUN sudo add-apt-repository ppa:kisak/kisak-mesa

RUN sudo apt install xterm -y

# Install Gazebo Harmonic, GZ ROS packages and rviz2 from offical binaries
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y \
  && sudo apt-get install -y gz-$GZ_VERSION \
  && sudo apt-get install -y ros-${ROS_DISTRO}-ros-gz \
  && sudo apt-get install -y ros-${ROS_DISTRO}-rviz2

ENV SPACEROS_USER spaceros-user

# Generate repos file for demo dependencies, excluding packages from Space ROS core.
COPY --chown=${SPACEROS_USER}:${SPACEROS_USER} demo-pkgs.txt /tmp/
COPY --chown=${SPACEROS_USER}:${SPACEROS_USER} excluded-pkgs.txt /tmp/
RUN rosinstall_generator \
  --rosdistro ${ROS_DISTRO} \
  --deps \
  --exclude-path ${SPACEROS_DIR}/src \
  --exclude $(cat /tmp/excluded-pkgs.txt) -- \
  -- $(cat /tmp/demo-pkgs.txt) \
  > /tmp/demo_generated_pkgs.repos

# Create workspace for lunar_terrain demo
RUN mkdir -p ${DEMO_DIR}/src
WORKDIR ${DEMO_DIR}
RUN vcs import src < /tmp/demo_generated_pkgs.repos

# Get the source for the dependencies
# RUN vcs import src < /tmp/demo_generated_pkgs.repos
COPY --chown=${SPACEROS_USER}:${SPACEROS_USER} demo_manual_pkgs.repos /tmp/
RUN vcs import src < /tmp/demo_manual_pkgs.repos

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y \
  && /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
  && source "${SPACEROS_DIR}/setup.bash" \
  && rosdep install \
     --simulate \
     --from-paths src \
     --ignore-src \
     -r -y \
     --rosdistro ${ROS_DISTRO} > /tmp/rosdep-commands.sh \
  && chmod +x /tmp/rosdep-commands.sh \
  && /tmp/rosdep-commands.sh'

# Copy the demo source code
COPY --chown=${SPACEROS_USER}:${SPACEROS_USER} lunar_sun_gz_plugin src/lunar_sun_gz_plugin
COPY --chown=${SPACEROS_USER}:${SPACEROS_USER} lunar_terrain_gz_bringup src/lunar_terrain_gz_bringup
COPY --chown=${SPACEROS_USER}:${SPACEROS_USER} lunar_terrain_gz_worlds src/lunar_terrain_gz_worlds

# Build the demo
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
  && source ${SPACEROS_DIR}/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Create the render group if it doesn't exist
RUN sudo groupadd -f render

# Add the user to the render group so that the user can access /dev/dri/renderD128
RUN sudo usermod -aG render ${SPACEROS_USER}

# Setup the entrypoint
COPY --chown=${SPACEROS_USER}:${SPACEROS_USER} ./entrypoint.sh /
# RUN chown appuser:appuser /entrypoint.sh && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
