# Copyright 2024 Azmyin Md. Kamal
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
# A Docker configuration script to build the o3de_curiosit_docker image that showcases Open 3D Engine
# in simulating a Curiositry rover traversing through a test ground inspired by the NASA JPL Mars Yard
# O3DE docker file based on https://github.com/husarion/o3de-docker/blob/main/Dockerfile
#
#
# The script provides the following build arguments:
#
#   VCS_REF     - The git revision of the Space ROS source code (no default value).
#   VERSION     - The version of Space ROS (default: "preview")

FROM openrobotics/space_robots_demo

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"
ARG ROS_DISTRO=humble 

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="O3DE Curiosity Rover"
LABEL org.label-schema.description="O3DE Curiosity rover demo image"
LABEL org.label-schema.vendor="Nasa Space ROS Sim Summer Spring Challenge 2024"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/Mechazo11/space-ros-docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Define a key enviornment variables
ENV MOVEIT2_DIR=${HOME_DIR}/moveit2
ENV DEMO_DIR=${HOME_DIR}/demos_ws
ENV IGNITION_VERSION fortress
ENV GZ_VERSION fortress
ENV PROJECT_NAME=RobotSim
ENV O3DE_ENGINE=${HOME_DIR}/o3de
ENV O3DE_DIR=${HOME_DIR}/O3DE
ENV O3DE_EXTRAS=${HOME_DIR}/o3de-extras
ENV PROJECT_PATH=${O3DE_DIR}/Projects/${PROJECT_NAME}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

### Delete CycloneDDS and install from apt to resolve cmake conflict ###

# Clear Install folder, we will need to rebuild the workspace again
WORKDIR ${SPACEROS_DIR}
RUN sudo rm -rf install/

# Delete problematic packages
WORKDIR ${SPACEROS_DIR}/src
RUN sudo rm -rf rmw_cyclonedds_cpp
RUN sudo rm -rf rmw_implementation  
RUN sudo rm -rf rmw_cyclonedds
RUN sudo rm -rf realtime_tools

WORKDIR ${SPACEROS_DIR}

# Update rosdep
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get dist-upgrade -y
RUN rosdep update

# TODO open a PR to resolve rmw-cyclonedds and rmw-implementation causing .
# Install them back from apt
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt install -y \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-rmw-implementation \
  ros-${ROS_DISTRO}-realtime-tools

# Rebuild Space ROS workspace
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash \ 
    && colcon build --packages-ignore test_rmw_implementation'

# Ensure proper permission for all files in SPACEROS_DIR
RUN sudo chown -R ${USERNAME}:${USERNAME} ${SPACEROS_DIR}

### Delete CycloneDDS and install from apt to resolve cmake conflict ###

### Delete DEMO_WS repo, redownload correct one build the DEMO_WS again ###
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  python3-rosinstall-generator \
  libboost-python-dev build-essential cmake libboost-all-dev libois-dev \
  libzzip-dev libfreeimage-dev libfreetype6-dev libx11-dev libxt-dev libxaw7-dev \
  libsdl2-dev libpugixml-dev libglew-dev qtbase5-dev libspdlog-dev libfmt-dev \
  libignition-gazebo6-dev libssl-dev devscripts debian-keyring fakeroot debhelper \
  libboost-dev libsasl2-dev libicu-dev libzstd-dev doxygen libtinyxml-dev \
  libtinyxml2-dev python3-pyqt5 tzdata locales keyboard-configuration

RUN sudo sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && sudo dpkg-reconfigure --frontend=noninteractive locales \
    && sudo update-locale LANG=en_US.UTF-8

    RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update -y && sudo apt-get install -y \
    bc bind9-utils binutils ca-certificates clang cmake \
    file firewalld git git-lfs jq kbd kmod less \
    lsb-release \
    libglu1-mesa-dev \
    libxcb-xinerama0 \
    libfontconfig1-dev \
    libcurl4-openssl-dev \
    libnvidia-gl-470 \
    libssl-dev \
    libxcb-xkb-dev \
    libxkbcommon-x11-dev \
    libxkbcommon-dev \
    libxcb-xfixes0-dev \
    libxcb-xinput-dev \
    libxcb-xinput0 \
    libpcre2-16-0 \
    lsof \
    net-tools \
    ninja-build \
    pciutils \
    python3-pip \
    software-properties-common \
    sudo \
    tar \
    unzip \
    vim \
    wget \
    xz-utils
    # && rm -rf /var/lib/apt/lists/* #  TODO need to figure out how to use this correctly
  

WORKDIR ${DEMO_DIR}
RUN sudo rm -rf install/
RUN sudo rm -rf build/
RUN sudo rm -rf log/



# Add the user to the render group so that the user can access /dev/dri/renderD128
# /dev/dri/renderD128 gives access to hardware-accelerated rendering tool such as GPU
RUN sudo usermod -aG render $USERNAME

# Setup the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
