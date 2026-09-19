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

FROM osrf/space-ros:latest AS base

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Mars Helicopter"
LABEL org.label-schema.description="Simulation of the Ingenuity mars helicopter in a Gazebo environment"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/demos"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Define a few key variables
ENV DEMO_DIR=${HOME_DIR}/demo_ws
ENV IGNITION_VERSION=fortress
ENV GZ_VERSION=fortress

# Define key locations
RUN mkdir -p ${DEMO_DIR}/src
WORKDIR ${DEMO_DIR}

# Define the XDG_RUNTIME_DIR environment variable
ENV XDG_RUNTIME_DIR=/run/${USERNAME}/1000

# Create the runtime directory, set appropriate ownership, and adjust permissions to 0700
RUN sudo mkdir -p ${XDG_RUNTIME_DIR} && \
    sudo chown 1000:1000 ${XDG_RUNTIME_DIR} && \
    sudo chmod 0700 ${XDG_RUNTIME_DIR}

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && sudo apt-get update && sudo apt-get install -y ignition-fortress

# development version
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
    && sudo apt-get update && sudo apt-get install -y libignition-gazebo6-dev ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ros-ign ros-humble-ign-ros2-control

FROM base AS workspace

# Build argument to bust cache
ARG CACHEBUST=1

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Get the source files
COPY --chown=${USERNAME}:${USERNAME} demo_manual_pkgs.repos /tmp/
RUN vcs import src < /tmp/demo_manual_pkgs.repos && /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' 
RUN cp -r src/demos/mars_helicopter src && rm -rf src/demos

RUN /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash" && source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && colcon build'

# Setup the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]