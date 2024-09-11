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

FROM openrobotics/moveit2:latest

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"
ARG ROS_DISTRO=humble 

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="O3DE Curiosity Rover"
LABEL org.label-schema.description="Curiosity rover demo on a new test enviornment on the Space ROS platform and Open 3D Engine"
LABEL org.label-schema.vendor="Nasa Space ROS Sim Summer Spring Challenge 2024"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/Mechazo11/space-ros-docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Define a key enviornment variables
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

####  Install DEMO_WS   ####

# Get rosinstall_generator
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
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

# RUN apt-get install -y tzdata
# RUN apt-get install -y locales
# RUN apt-get install -y keyboard-configuration

RUN sudo sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && sudo dpkg-reconfigure --frontend=noninteractive locales \
    && sudo update-locale LANG=en_US.UTF-8

# RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
# && dpkg-reconfigure --frontend=noninteractive locales \
# && update-locale LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y && sudo apt-get install -y \
  bc \
  bind9-utils \
  binutils \
  ca-certificates \
  clang \
  cmake \
  file \
  firewalld \
  git \
  git-lfs \
  jq \
  kbd \
  kmod \
  less \
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


RUN mkdir -p ${DEMO_DIR}/src
WORKDIR ${DEMO_DIR}

# Install libmongoc for development
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get install libmongoc-dev -y

# Compile mongo cxx driver https://mongocxx.org/mongocxx-v3/installation/linux/
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get install libssl-dev build-essential devscripts debian-keyring fakeroot debhelper cmake libboost-dev libsasl2-dev libicu-dev libzstd-dev doxygen -y
RUN wget https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.6.7/mongo-cxx-driver-r3.6.7.tar.gz
RUN tar -xzf mongo-cxx-driver-r3.6.7.tar.gz
RUN cd mongo-cxx-driver-r3.6.7/build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && sudo cmake --build . --target EP_mnmlstc_core && cmake --build . && sudo cmake --build . --target install

# Get the source for the dependencies
# RUN vcs import src < /tmp/demo_generated_pkgs.repos
COPY --chown=${USERNAME}:${USERNAME} o3de-robot-sim.repos /tmp/
RUN vcs import src < /tmp/o3de-robot-sim.repos && /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' 

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y \
&& /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' \
&& /bin/bash -c 'source "${MOVEIT2_DIR}/install/setup.bash"' \
&& rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROSDISTRO}

# Build the demo_ws
RUN /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash && source ${MOVEIT2_DIR}/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Create O3DE/Projects and O3DE/Gems directory
RUN if [ -d "${O3DE_DIR}/Projects" ]; then rm -rf ${O3DE_DIR}/Gems; fi
RUN mkdir -p ${O3DE_DIR}/Projects

RUN if [ -d "${O3DE_DIR}/Gems" ]; then rm -rf ${O3DE_DIR}/Gems; fi
RUN mkdir -p ${O3DE_DIR}/Gems

# Git clone RobotSim, NasaMarsRoverGem and MarsYardGem
WORKDIR ${O3DE_DIR}/Projects
RUN git clone --branch nasa_submission https://github.com/Mechazo11/RobotSim.git

WORKDIR ${O3DE_DIR}/Gems
RUN git clone --branch nasa_submission https://github.com/Mechazo11/NasaCuriosityRoverGem.git
RUN git clone --branch nasa_submission https://github.com/Mechazo11/MarsYardGem.git

## Symlink clang version to non-versioned clang and set cc to clang
# RUN update-alternatives --install /usr/bin/cc cc /usr/bin/clang 100 \
#     && update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++ 100

####  Install Open 3D Engine   ####  
# Install dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update -y && sudo apt-get install -y \
    bc \
    bind9-utils \
    binutils \
    ca-certificates \
    clang \
    cmake \
    file \
    firewalld \
    git \
    git-lfs \
    jq \
    kbd \
    kmod \
    less \
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

# Pull latest commit from development and build o3de. Note this does't build the whole engine
WORKDIR ${HOME_DIR}

# Install o3de engine (automatically pulls last commit from "development" branch)
RUN git clone --single-branch -b development https://github.com/o3de/o3de.git \
    && cd o3de \
    && git lfs install \
    && git lfs pull \
    && python/get_python.sh

# Install o3de-extras (automatically pulls last commit from "development" branch)
# Contains ROS 2 Gem and other relevant assets

WORKDIR ${HOME_DIR}
RUN git clone --single-branch -b development https://github.com/o3de/o3de-extras.git \
    && cd o3de-extras \
    && git lfs install \
    && git lfs pull

WORKDIR ${HOME_DIR}/o3de

# Register RobotSim with this engine
#RUN ${O3DE_ENGINE}/scripts/o3de.sh register --this-engine 
RUN ./scripts/o3de.sh register --this-engine 
# Register ROS 2 Gem, Husarion robot, Curiosity rover and Mars Yard gem
# RUN ${O3DE_ENGINE}/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS}/Gems/ROS2 \
#     && ${O3DE_ENGINE}/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS}/Gems/RosRobotSample \
#     && ${O3DE_ENGINE}/scripts/o3de.sh register --gem-path ${O3DE_DIR}/Gems/NasaCuriosityRoverGem \
#     && ${O3DE_ENGINE}/scripts/o3de.sh register --gem-path ${O3DE_DIR}/Gems/MarsYardGem

RUN ./scripts/o3de.sh register --gem-path ${O3DE_EXTRAS}/Gems/ROS2 \
    && ./scripts/o3de.sh register --gem-path ${O3DE_EXTRAS}/Gems/RosRobotSample \
    && ./scripts/o3de.sh register --gem-path ${O3DE_EXTRAS}/Gems/WarehouseSample \
    && ./scripts/o3de.sh register --gem-path ${O3DE_DIR}/Gems/NasaCuriosityRoverGem \
    && ./scripts/o3de.sh register --gem-path ${O3DE_DIR}/Gems/MarsYardGem

# Register the Terrian gem which is located in o3de folder
RUN ./scripts/o3de.sh register --gem-path ${HOME_DIR}/o3de/Gems/Terrain

# Register RobotSim project.
# Project must be registered at the very end after all dependent gems are registered
#RUN {O3DE_ENGINE}/scripts/o3de.sh register --project-path ${O3DE_DIR}/Projects/${PROJECT_NAME}
RUN ./scripts/o3de.sh register --project-path ${O3DE_DIR}/Projects/${PROJECT_NAME}

# Move to project directory
WORKDIR ${HOME_DIR}
# Run cmake and build project (Warning takes about 10-15 mins)

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update -y && sudo apt-get install -y \
    libunwind-dev

# Testing build by manually building out a ROS 2 Humble core
# RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
#     --mount=type=cache,target=/var/lib/apt,sharing=locked \
#     sudo apt-get update -y && sudo apt-get install -y \
#     ros-humble-ros-base

# # Move to project directory
WORKDIR ${PROJECT_PATH}

# Activate Cyclone DDS
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 

# Source space_ros, movit2 and demo_ws
RUN /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' \
  && /bin/bash -c 'source "${MOVEIT2_DIR}/install/setup.bash"' \
  && /bin/bash -c 'source "${DEMO_DIR}/install/setup.bash"'
  #&& cmake -B build/linux -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_STRIP_DEBUG_SYMBOLS=ON -DAZ_USE_PHYSX5:BOOL=ON \
  #&& cmake --build build/linux --config profile --target ${PROJECT_NAME} Editor ${PROJECT_NAME}.Assets

#RUN /bin/bash -c 'source "/opt/ros/humble/setup.bash"'

RUN cmake -B build/linux -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_STRIP_DEBUG_SYMBOLS=ON -DAZ_USE_PHYSX5:BOOL=ON \
  && cmake --build build/linux --config profile --target ${PROJECT_NAME} Editor ${PROJECT_NAME}.Assets




# # Add the user to the render group so that the user can access /dev/dri/renderD128
# # /dev/dri/renderD128 gives access to hardware-accelerated rendering tool such as GPU
# RUN sudo usermod -aG render $USERNAME

# # Setup the entrypoint
# COPY ./entrypoint.sh /
# ENTRYPOINT ["/entrypoint.sh"]
# CMD ["bash"]
