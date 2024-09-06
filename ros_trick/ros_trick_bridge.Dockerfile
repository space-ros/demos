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

# Tested with humble-2024.07.0 release
FROM osrf/space-ros:latest

# ------------------------------------------------------------------------------
# Get Install trick dependencies
# ------------------------------------------------------------------------------
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \ 
  sudo apt-get update && sudo apt-get install -y \
  bison \
  clang \
  flex \
  git \
  llvm \
  make \
  maven \
  swig \
  cmake \
  curl \
  g++ \
  libx11-dev \
  libxml2-dev \
  libxt-dev \
  libmotif-common \
  libmotif-dev \
  python3-dev \
  zlib1g-dev \
  llvm-dev \
  libclang-dev \
  libudunits2-dev \
  libgtest-dev \
  openjdk-11-jdk \
  zip

ENV PYTHON_VERSION=3

# ------------------------------------------------------------------------------
# Get Trick version 19.7.2 from GitHub, configure and build it.
# ------------------------------------------------------------------------------
WORKDIR /opt/trick
RUN git clone --branch 19.7.2 --depth 1 https://github.com/nasa/trick.git .
# cd into the directory we just created and ..
# configure and make Trick.
RUN ./configure && make

# ------------------------------------------------------------------------------
# Add ${TRICK_HOME}/bin to the PATH variable.
# ------------------------------------------------------------------------------
ENV TRICK_HOME="/opt/trick"
RUN echo "export PATH=${PATH}:${TRICK_HOME}/bin" >> ~/.bashrc


# ------------------------------------------------------------------------------
# Build SPACEROS workspace with ros trick bridge
# ------------------------------------------------------------------------------
WORKDIR /opt/ros_trick_bridge_ws
ENV ROS_TRICK_BRIDGE_WS="/opt/ros_trick_bridge_ws/"
COPY --chown=spaceros-user:spaceros-user ros_src/ros_trick_bridge src/
RUN /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers desktop_notification- status-'
RUN rm -rf build log src

# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# Parts below are only required for the canadarm demo. You can replace them as you see fit.
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Install RBDL, which is used for calculating forward dynamics in trick
# ------------------------------------------------------------------------------
WORKDIR /opt/rbdl
RUN git clone --branch v3.3.0 --depth 1 https://github.com/rbdl/rbdl.git . \
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
# make it easy to link the rbdl library.
# LD_LIBRARY_PATH is empty at this stage of dockerbuild
ENV LD_LIBRARY_PATH="/usr/local/lib"

# ------------------------------------------------------------------------------
# copy the created canadarm trick simulation and compile it
# ------------------------------------------------------------------------------
WORKDIR /opt/trick_sims/SIM_trick_canadarm
COPY --chown=spaceros-user:spaceros-user trick_src/SIM_trick_canadarm/ .
RUN ${TRICK_HOME}/bin/trick-CP
# include canadarm URDF for RBDL
COPY --chown=spaceros-user:spaceros-user ros_src/trick_canadarm_moveit_config/config/SSRMS_Canadarm2.urdf.xacro .

WORKDIR ${ROS_TRICK_BRIDGE_WS}
