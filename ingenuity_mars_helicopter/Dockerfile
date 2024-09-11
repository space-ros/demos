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

FROM osrf/space-ros:latest

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

ENV INGENUITY_DIR=${HOME_DIR}/ingenuity_helicopter

RUN sudo apt-get update && \
    DEBIAN_FRONTEND=noninteractive sudo apt-get install -y --no-install-recommends \
    ros-humble-hardware-interface \
    ros-humble-generate-parameter-library \
    ros-humble-xacro \
    ros-humble-controller-interface \
    ros-humble-realtime-tools \
    ros-humble-ros2-control-test-assets \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher-gui \
    ros-humble-control-msgs \
    ros-humble-gazebo-ros \
    ros-humble-control-toolbox \
    ros-humble-ackermann-msgs \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-clang-format \
    ros-humble-moveit \
    ros-humble-joint-state-broadcaster \
    libignition-gazebo6-dev \
    ros-humble-joint-trajectory-controller \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-gz-bridge \
    libeigen3-dev

# Install the various build and test tools
RUN sudo apt install -y \
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

# Install some pip packages needed for testing
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

COPY ingenuity_mars_helicopter ${INGENUITY_DIR}/src/ingenuity_mars_helicopter
COPY helicopter_flight_control ${INGENUITY_DIR}/src/helicopter_flight_control
COPY helicopter_flight_simulation ${INGENUITY_DIR}/src/helicopter_flight_simulation
COPY ingenuity_bringup ${INGENUITY_DIR}/src/ingenuity_bringup
COPY ingenuity_description ${INGENUITY_DIR}/src/ingenuity_description
COPY ./entrypoint.sh ${INGENUITY_DIR}/entrypoint.sh

RUN sudo chown -R ${USERNAME}:${USERNAME} ${INGENUITY_DIR}

RUN sudo chmod +x ${INGENUITY_DIR}/entrypoint.sh

ENTRYPOINT ["/home/spaceros-user/ingenuity_helicopter/entrypoint.sh"]
