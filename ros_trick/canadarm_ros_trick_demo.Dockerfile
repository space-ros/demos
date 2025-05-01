# Copyright 2024 Blazej Fiderek (xfiderek)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# The base image is built from
# https://github.com/space-ros/docker/blob/jazzy-2025.04.0/space_robots/
# Git tag: jazzy-2025.04.0.
FROM openrobotics/space_robots_demo:latest

# Update the ROS package keys
ADD --chmod=644 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /usr/share/keyrings/ros-archive-keyring.gpg

# Rviz does not work inside the newest space_robots_demo, so we have to upgrade
# the following packages. These packages are already present in the base image
# and here we are upgrading them to latest versions.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update && sudo apt-get install -y \
    mesa-libgallium \
    libdrm-amdgpu1 \
    libdrm-common \
    libdrm-intel1 \
    libdrm-radeon1 \
    libdrm2 \
    libegl-mesa0 \
    libgbm1 \
    libgl1-mesa-dev \
    libglapi-mesa \
    libglx-mesa0 \
    liborc-0.4-0 \
    libpq5 \
    linux-libc-dev \
    mesa-va-drivers \
    mesa-vdpau-drivers \
    mesa-vulkan-drivers

ENV TRICK_DEMO_WS=${HOME_DIR}/ros_trick_demo_ws

RUN mkdir ${TRICK_DEMO_WS}
WORKDIR ${TRICK_DEMO_WS}
COPY --chown=spaceros-user:spaceros-user ros_src ${TRICK_DEMO_WS}/src

RUN /bin/bash -c 'source ${DEMO_DIR}/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers desktop_notification- status-'
RUN rm -rf build log src
