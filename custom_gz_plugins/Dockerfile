# Copyright 2024 Open Source Robotics Foundation, Inc.
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

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Real World Extraterrestrial Simulations"
LABEL org.label-schema.description="Realistic Extraterrestrial Simulations demos using custom GZ plugins & data from NASA PDS"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Define a few key variables
ENV DEMO_DIR=${HOME_DIR}/demos_ws
ENV GZ_VERSION=harmonic

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Clone all space-ros sources
RUN mkdir ${SPACEROS_DIR}/src \
  && vcs import ${SPACEROS_DIR}/src < ${SPACEROS_DIR}/exact.repos

# Make sure the latest versions of packages are installed
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get dist-upgrade -y
RUN rosdep update

# Get rosinstall_generator
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y && sudo apt-get install -y python3-rosinstall-generator

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

# Add osrf packages repository and manually install gz sim harmonic dependecies as they haven't been added to the rosdep index as of yet
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update && sudo apt-get install -y libgz-math7-dev libgz-transport13-dev libgz-sim8-dev libgz-msgs10-dev libgz-plugin2-dev

# Get the source for the dependencies
COPY --chown=${USERNAME}:${USERNAME} dependencies_source_pkgs.repos /tmp/
RUN vcs import src < /tmp/dependencies_source_pkgs.repos && /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' 

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
--mount=type=cache,target=/var/lib/apt,sharing=locked \
sudo apt-get update -y \
&& /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' \
&& rosdep install --from-paths ${SPACEROS_DIR}/src src --ignore-src -r -y --rosdistro ${ROSDISTRO} --skip-keys "console_bridge generate_parameter_library fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers rmw_connextdds ros_testing rmw_connextdds rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp composition demo_nodes_py lifecycle rosidl_typesupport_fastrtps_cpp rosidl_typesupport_fastrtps_c ikos diagnostic_aggregator diagnostic_updater joy qt_gui rqt_gui rqt_gui_py"

# Build the demo
RUN /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Add the user to the render group so that the user can access /dev/dri/renderD128
RUN sudo usermod -aG render $USERNAME

# Setup the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
