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

FROM osrf/space-ros:jazzy-2026.07.0

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"
ARG ROSDISTRO="jazzy"
ARG USERNAME="spaceros-user"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Lunar Pole Exploration Rover"
LABEL org.label-schema.description="Lunar Pole Exploration Rover entry for the NASA Space ROS Sim Summer Sprint Challenge"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/demos"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Update the ROS package keys
ADD --chmod=644 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /usr/share/keyrings/ros-archive-keyring.gpg

# Make sure the latest versions of packages are installed
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get dist-upgrade -y

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
  python3-rosdep \
  python3-rosinstall-generator \
  python3-setuptools \
  python3-vcstool \
  wget

# Define a few key variables
ENV DEMO_DIR=${HOME}/demos_ws
ENV GZ_VERSION=harmonic
# Space ROS is built with CycloneDDS only; the Jazzy binaries would default to Fast DDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Define key locations
RUN mkdir -p ${DEMO_DIR}/src
WORKDIR ${DEMO_DIR}

COPY . src/lunar_pole_exploration_rover
RUN sudo rosdep init && rosdep update

# Get the source for the simulation assets (rover model, terrain, power plugins).
# We use lfs specifically to pull resources from the simulation repo.
COPY --chown=${USERNAME}:${USERNAME} demo_manual_pkgs.repos /tmp/
RUN vcs import src < /tmp/demo_manual_pkgs.repos \
  && vcs custom src --args lfs pull

# Everything else the demo needs (Gazebo Harmonic, ros_gz, ros2_control and the
# controllers) comes from the ROS 2 Jazzy binary packages, resolved through rosdep.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y \
&& rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROSDISTRO}

ENV MAKEFLAGS="-j1"

# build demo
RUN /bin/bash -c 'source /opt/ros/${ROSDISTRO}/setup.bash \
  && source ${SPACEROS_DIR}/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers desktop_notification- status-'

# Create the render group if it doesn't exist
RUN sudo groupadd -f render

# Add the user to the render group so that the user can access /dev/dri/renderD128
RUN sudo usermod -aG render $USERNAME

# Setup the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
