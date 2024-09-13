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

FROM osrf/space-ros:latest

# Define build arguments and metadata
ARG VCS_REF
ARG VERSION="preview"
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="CADRE demo: Cooperative Autonomous Distributed Robotic Exploration"
LABEL org.label-schema.description="Three small rovers for NASA Space ROS Sim Summer Sprint Challenge"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable interactive prompts for package installations
ARG DEBIAN_FRONTEND=noninteractive

# Define environment variables for Classic Gazebo
ENV DEMO_DIR=${HOME_DIR}/demos_ws
ENV GAZEBO_VERSION=11 
ENV GAZEBO_MODEL_PATH=/root/.gazebo/models
ENV GAZEBO_RESOURCE_PATH=/root/.gazebo
ENV GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
ENV GAZEBO_MASTER_URI=http://localhost:11345
ENV GAZEBO_IP=127.0.0.1
ENV ROS_DISTRO=humble
USER root
# Prepare the workspace
RUN mkdir -p ${DEMO_DIR}/src
WORKDIR ${DEMO_DIR}

# Update and upgrade the base system
RUN apt-get update && apt-get dist-upgrade -y

# Install ROS dependencies and tools
RUN apt-get install -y \
  build-essential \
  clang-format \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

# Install Gazebo 11 (Classic Gazebo) and ROS 2 integration packages
RUN apt-get update && apt-get install -y \
  gazebo11 \
  libgazebo11-dev \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs \
  ros-${ROS_DISTRO}-gazebo-ros2-control \
  ros-${ROS_DISTRO}-gazebo-ros2-controllers \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-xacro \
  python3-vcstool \
  libbullet-dev \
  xterm \
  vim

# Install Python testing tools
RUN python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

# Install rosinstall_generator
RUN apt-get install -y python3-rosinstall-generator

# Generate the repos file for demo dependencies
COPY --chown=${USERNAME}:${USERNAME} demo-pkgs.txt /tmp/
COPY --chown=${USERNAME}:${USERNAME} excluded-pkgs.txt /tmp/
RUN rosinstall_generator \
  --upstream \
  --rosdistro ${ROS_DISTRO} \
  --deps \
  --exclude-path ${SPACEROS_DIR}/src \
  --exclude $(cat /tmp/excluded-pkgs.txt) -- \
  -- $(cat /tmp/demo-pkgs.txt) \
  > /tmp/demo_generated_pkgs.repos

# Import dependencies from the generated repos
RUN vcs import src < /tmp/demo_generated_pkgs.repos

# Import additional manual dependencies
COPY --chown=${USERNAME}:${USERNAME} demo_manual_pkgs.repos /tmp/
RUN rm -rf ${DEMO_DIR}/src/* && vcs import src < /tmp/demo_manual_pkgs.repos

# Source ROS setup and install dependencies
RUN apt-get update -y && \
  source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# Build the demo
ENV MAKEFLAGS="-j1"
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Add the user to the render group for GPU access
RUN usermod -aG render ${USERNAME}

# Install sample GUI apps for testing
RUN apt-get install -y \
  firefox \
  glmark2 \
  libcanberra-gtk3-0 \
  libpci-dev \
  xauth \
  xterm

# Setup entrypoint and run command
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
