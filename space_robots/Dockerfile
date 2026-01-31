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

FROM osrf/space-ros-moveit2:jazzy-2026.01.0

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"
ARG USERNAME="spaceros-user"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Curiosity Rover"
LABEL org.label-schema.description="Curiosity rover demo on the Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Define a few key variables
ENV DEMO_DIR=${HOME}/demos_ws
WORKDIR ${DEMO_DIR}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Update the ROS package keys
ADD --chmod=644 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /usr/share/keyrings/ros-archive-keyring.gpg

# Install base image dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update -y && \
    sudo -E apt-get install -y \
      git-lfs \
      python3-rosinstall-generator

RUN git clone https://github.com/space-ros/demos.git ${DEMO_DIR}/src/demos
RUN git clone https://github.com/space-ros/simulation.git ${DEMO_DIR}/src/simulation

# Space Robots and RACS2 both reuse other demo assets and can lead to conflicts
# This is a temporary fix for [issue #130](https://github.com/space-ros/demos/issues/130)
RUN rm -r ${DEMO_DIR}/src/demos/racs2_demos_on_spaceros
# the next 2 demos are not related to space_robots so we drop them from this build to not conflict on versions
RUN rm -r ${DEMO_DIR}/src/demos/lunar_terrain
RUN rm -r ${DEMO_DIR}/src/demos/ros_trick


# Get a list of all installed ros2 packges
RUN source "${SPACEROS_DIR}/setup.bash" && \
    source "${MOVEIT2_DIR}/install/setup.bash" && \
    ros2 pkg list > /tmp/installed-pkgs.txt

# Generate repos file for demo dependencies, excluding previously installed packages
COPY --chown=${USERNAME}:${USERNAME} demo-pkgs.txt /tmp/
COPY --chown=${USERNAME}:${USERNAME} excluded-pkgs.txt /tmp/
RUN rosinstall_generator \
  --rosdistro ${ROS_DISTRO} \
  --deps \
  --exclude $(cat /tmp/excluded-pkgs.txt /tmp/installed-pkgs.txt) -- \
  $(cat /tmp/demo-pkgs.txt) \
  > demo_generated_pkgs.repos

RUN vcs import --shallow src < demo_generated_pkgs.repos

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    sudo apt-get update -y && \
    source "${SPACEROS_DIR}/setup.bash" && \
    source "${MOVEIT2_DIR}/install/setup.bash" && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# Build the demo
RUN source ${SPACEROS_DIR}/setup.bash && \
    source ${MOVEIT2_DIR}/install/setup.bash && \
    colcon build \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release

# Setup the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
