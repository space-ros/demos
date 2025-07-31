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

FROM osrf/space-ros:main 

# Update the ROS package keys
ADD --chmod=644 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /usr/share/keyrings/ros-archive-keyring.gpg

# Install trick dependencies.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update && sudo apt-get install -y \
    bison \
    clang \
    cmake \
    curl \
    flex \
    g++ \
    git \
    libclang-dev \
    libgtest-dev \
    libmotif-common \
    libmotif-dev \
    libudunits2-dev \
    libx11-dev \
    libxml2-dev \
    libxt-dev \
    llvm \
    llvm-dev \
    make \
    maven \
    openjdk-11-jdk \
    python3-dev \
    swig \
    zip \
    zlib1g-dev

ENV PYTHON_VERSION=3

# Note: For Jazzy, we need a fix in TRICK to work in 24.04 (commit 4781cfe)
# there is not yet a tag including the fix commit, checking out the latest one tested to work
RUN mkdir ${HOME}/trick
WORKDIR ${HOME}/trick
RUN git clone --branch master https://github.com/nasa/trick.git . && git checkout 1b6d75b
RUN ./configure && make

# Add ${TRICK_HOME}/bin to the PATH variable.
ENV TRICK_HOME="${HOME}/trick"
RUN echo "export PATH=${PATH}:${TRICK_HOME}/bin" >> ~/.bashrc

# Build SPACEROS workspace with ros trick bridge.
RUN mkdir ${HOME}/ros_trick_bridge_ws
WORKDIR ${HOME}/ros_trick_bridge_ws
ENV ROS_TRICK_BRIDGE_WS="${HOME}/ros_trick_bridge_ws/"
COPY --chown=spaceros-user:spaceros-user ros_src/ros_trick_bridge src/
RUN /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers desktop_notification- status-'
RUN rm -rf build log src

# The parts below are only required for the canadarm demo. Replace them as you
# see fit.

# Install RBDL, which is used for calculating forward dynamics in trick.
RUN mkdir ${HOME}/rbdl
WORKDIR ${HOME}/rbdl
RUN git clone --branch v3.3.1 --depth 1 https://github.com/rbdl/rbdl.git . \
  && git submodule update --init --remote --depth 1 addons/urdfreader/
RUN   mkdir ./rbdl-build \
  && cd rbdl-build/ \
  && cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DRBDL_BUILD_ADDON_URDFREADER="ON" \
  .. \
  && make \
  && sudo make install \
  && cd .. \
  && rm -r ./rbdl-build
# Make it easy to link the rbdl library.
# LD_LIBRARY_PATH is empty at this stage of dockerbuild
ENV LD_LIBRARY_PATH="/usr/local/lib"

# Copy the created canadarm trick simulation and compile it.
RUN mkdir -p ${HOME}/trick_sims/SIM_trick_canadarm
WORKDIR ${HOME}/trick_sims/SIM_trick_canadarm
COPY --chown=spaceros-user:spaceros-user trick_src/SIM_trick_canadarm/ .
RUN ${TRICK_HOME}/bin/trick-CP
# Include canadarm URDF for RBDL
COPY --chown=spaceros-user:spaceros-user ros_src/trick_canadarm_moveit_config/config/SSRMS_Canadarm2.urdf.xacro .

WORKDIR ${ROS_TRICK_BRIDGE_WS}
