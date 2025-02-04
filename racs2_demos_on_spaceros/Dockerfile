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

FROM openrobotics/space_robots_demo:latest

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="RACS2 demo on Curiosity Rover"
LABEL org.label-schema.description="RACS2 demo on the Space ROS platform"
LABEL org.label-schema.vendor="JAXA"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Define a few key variables
ENV RACS2_DEMO_DIR=${HOME_DIR}/racs2_ws

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Get rosinstall_generator
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y && sudo apt-get install -y python3-rosinstall-generator

RUN mkdir -p ${RACS2_DEMO_DIR}
WORKDIR ${RACS2_DEMO_DIR}

# Install dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get install libwebsockets-dev libwebsockets-dev protobuf-c-compiler libprotobuf-c-dev python3-pip tmux gdb -y
RUN python3 -m pip install protobuf==3.20.0 websockets==12.0

# Get the cFS source code
RUN git clone --recursive -b v6.7.0a https://github.com/nasa/cFS/ cfs
WORKDIR ${RACS2_DEMO_DIR}/cfs
RUN git submodule init
RUN git submodule update

# Get the RACS2 source code
WORKDIR ${RACS2_DEMO_DIR}
RUN git clone https://github.com/jaxa/racs2_bridge -b v1.1

# Get the demo source code
# rename old demo directory and exclude from build
WORKDIR ${DEMO_DIR}/src
# RUN mv demos demos_old
RUN cp -r demos demos_old
RUN touch demos_old/COLCON_IGNORE
# git clone new demo
# RUN git clone https://github.com/tt-saito/demos.git -b racs2_demo2
# RUN git clone https://github.com/space-ros/demos

# Customize cFS to run the bridge
WORKDIR ${RACS2_DEMO_DIR}/cfs
RUN cp cfe/cmake/Makefile.sample Makefile
RUN cp -r cfe/cmake/sample_defs sample_defs
RUN cp -pr ${RACS2_DEMO_DIR}/racs2_bridge/cFS/Bridge/Client_C/apps/racs2_bridge_client apps/

# Deploy the run_app application and adjust the startup scripts
# RUN cp -pr ${DEMO_DIR}/src/demos/racs2_demos_on_spaceros/cFS/sample_defs/* ${RACS2_DEMO_DIR}/cfs/sample_defs/
# RUN cp -pr ${DEMO_DIR}/src/demos/racs2_demos_on_spaceros/cFS/apps/run_app ${RACS2_DEMO_DIR}/cfs/apps/
COPY cFS/sample_defs/cpu1_cfe_es_startup.scr  ${RACS2_DEMO_DIR}/cfs/sample_defs/cpu1_cfe_es_startup.scr
COPY cFS/sample_defs/racs2_bridge_config.txt ${RACS2_DEMO_DIR}/cfs/sample_defs/racs2_bridge_config.txt
COPY cFS/sample_defs/targets.cmake  ${RACS2_DEMO_DIR}/cfs/sample_defs/targets.cmake
COPY cFS/apps/run_app  ${RACS2_DEMO_DIR}/cfs/apps/run_app

# This is necessary to run cFS inside docker
RUN sed -i -e 's/^#undef OSAL_DEBUG_PERMISSIVE_MODE/#define OSAL_DEBUG_PERMISSIVE_MODE 1/g' sample_defs/default_osconfig.h
RUN sed -i -e 's/^#undef OSAL_DEBUG_DISABLE_TASK_PRIORITIES/#define OSAL_DEBUG_DISABLE_TASK_PRIORITIES 1/g' sample_defs/default_osconfig.h

# This is only needed because docker by default starts in IPv4. This setting
# is specific to the JAXA bridge.
RUN sed -i -e 's/^wss_uri=.*/wss_uri=127.0.0.1/g' sample_defs/racs2_bridge_config.txt

# Compile cFS
RUN make SIMULATION=native prep
RUN make
RUN make install

# Prepare ROS packages
WORKDIR ${DEMO_DIR}
# Copy bridge & racs2_msg node
RUN cp -pr ${RACS2_DEMO_DIR}/racs2_bridge/ROS2/Bridge/Server_Python/bridge_py_s src/
RUN cp -pr ${RACS2_DEMO_DIR}/racs2_bridge/Example/Case.1/ROS2/racs2_msg src/


COPY  CMakeLists.txt  ${DEMO_DIR}/src/demos/mars_rover/CMakeLists.txt
COPY  package.xml  ${DEMO_DIR}/src/demos/mars_rover/package.xml
# COPY  racs2_demos_on_spaceros ${DEMO_DIR}/src/demos/mars_rover/racs2_demos_on_spaceros
COPY  launch/mars_rover.launch.py ${DEMO_DIR}/src/demos/mars_rover/launch/mars_rover.launch.py
COPY  nodes/RACS2Bridge_geometry_msgs_pb2.py ${DEMO_DIR}/src/demos/mars_rover/nodes/RACS2Bridge_geometry_msgs_pb2.py
COPY  nodes/move_wheel ${DEMO_DIR}/src/demos/mars_rover/nodes/move_wheel




# Install dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y \
&& /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' \
&& /bin/bash -c 'source "${MOVEIT2_DIR}/install/setup.bash"' \
&& rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# Build the demo
RUN /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash && source ${MOVEIT2_DIR}/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Add the user to the render group so that the user can access /dev/dri/renderD128
RUN sudo usermod -aG render $USERNAME

# Setup the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
