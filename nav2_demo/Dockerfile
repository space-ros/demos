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
# A Docker configuration script to build the Navigation2/Space ROS image.
#
# The script provides the following build arguments:
#
#   VCS_REF     - The git revision of the Space ROS source code (no default value).
#   VERSION     - The version of Space ROS (default: "preview")

FROM osrf/space-ros-nav2:jazzy-2026.07.0

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Navigation2 demo"
LABEL org.label-schema.description="Preview version of the Navigation2/Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Define workspace locations
ENV NAVIGATION2_WS=${HOME}/nav2_ws
ENV NAV2_DEMO_WS=${HOME}/nav2_demo_ws

# Build only what RViz needs beyond the Space ROS Nav2 image:
# - nav2_rviz_plugins (excluded from osrf/space-ros-nav2 to avoid RViz/Gazebo bloat)
# - nav2_demo_rviz (launch + default RViz config; replaces apt nav2_bringup)
# Match the Nav2 release used by the base image (see navigation2.repos).
ARG NAV2_RVIZ_PLUGINS_VERSION=1.3.12
RUN mkdir -p ${NAV2_DEMO_WS}/src \
    && git clone --depth 1 --filter=blob:none --sparse \
      --branch "${NAV2_RVIZ_PLUGINS_VERSION}" \
      https://github.com/ros-navigation/navigation2.git /tmp/navigation2 \
    && cd /tmp/navigation2 \
    && git sparse-checkout set nav2_rviz_plugins \
    && mv /tmp/navigation2/nav2_rviz_plugins ${NAV2_DEMO_WS}/src/ \
    && rm -rf /tmp/navigation2

COPY nav2_demo_rviz ${NAV2_DEMO_WS}/src/nav2_demo_rviz

WORKDIR ${NAV2_DEMO_WS}
# Source Space ROS Nav2 (cyclonedds), then add apt RViz prefixes without sourcing
# /opt/ros/jazzy/setup.bash (that underlay defaults to Fast DDS and breaks the build).
RUN /bin/bash -c "\
  source /opt/ros/spaceros/setup.bash && \
  source ${NAVIGATION2_WS}/install/setup.bash && \
  export CMAKE_PREFIX_PATH=/opt/ros/\${ROS_DISTRO}:\${CMAKE_PREFIX_PATH} && \
  export AMENT_PREFIX_PATH=/opt/ros/\${ROS_DISTRO}:\${AMENT_PREFIX_PATH} && \
  colcon build --symlink-install \
    --packages-select nav2_rviz_plugins nav2_demo_rviz"

# Setup mars_rover demo
# Install map file for localization
COPY mars_map* ${HOME}/nav2_ws

# Install nav2 config file for mars rover demo
COPY nav2_params.yaml ${HOME}/nav2_ws

WORKDIR ${NAVIGATION2_WS}

# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
