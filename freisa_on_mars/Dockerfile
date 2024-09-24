# ==========================================================================
# File: freisa_on_mars/Dockerfile
#
# Adapted from
# * https://github.com/space-ros/docker/blob/main/moveit2/Dockerfile
# * https://github.com/space-ros/docker/blob/main/space_robots/Dockerfile
# ==========================================================================

FROM osrf/space-ros:latest
# FROM openrobotics/moveit2:latest

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="FREISA meets Curiosity Rover"
LABEL org.label-schema.description="FREISA Robot Dog meets Curiosity rover demo on the Space ROS platform"
LABEL org.label-schema.vendor="B-AROL-O Team"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/demos"
LABEL org.label-schema.vcs-ref=${VCS_REF}

# Define a few key variables
ENV DEMO_DIR=${HOME_DIR}/demos_ws
ENV IGNITION_VERSION=fortress
ENV GZ_VERSION=fortress

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# === BEGIN Adaptation from space-ros/docker/moveit2/Dockerfile ===

# Define key locations
ENV MOVEIT2_DIR=${HOME_DIR}/moveit2

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

# Get the MoveIt2 source code
WORKDIR ${HOME_DIR}
RUN git clone https://github.com/moveit/moveit2.git -b ${ROSDISTRO} moveit2/src
RUN cd ${MOVEIT2_DIR}/src \
  && git clone https://github.com/moveit/moveit2_tutorials.git -b ${ROSDISTRO}

# Get rosinstall_generator
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y && sudo apt-get install -y python3-rosinstall-generator

RUN mkdir -p ${SPACEROS_DIR}/src # HACK

RUN touch DEBUG-01.txt

# Generate repos file for moveit2 dependencies, excluding packages from Space ROS core.
WORKDIR ${MOVEIT2_DIR}
COPY --chown=${USERNAME}:${USERNAME} moveit2-pkgs.txt /tmp/
COPY --chown=${USERNAME}:${USERNAME} excluded-pkgs.txt /tmp/
RUN rosinstall_generator \
  --rosdistro ${ROSDISTRO} \
  --deps \
  --exclude-path ${SPACEROS_DIR}/src \
  --exclude $(cat /tmp/excluded-pkgs.txt) -- \
  -- $(cat /tmp/moveit2-pkgs.txt) \
  > /tmp/moveit2_generated_pkgs.repos
  
# Get the repositories required by MoveIt2, but not included in Space ROS
WORKDIR ${MOVEIT2_DIR}
RUN vcs import src < /tmp/moveit2_generated_pkgs.repos
COPY --chown=${USERNAME}:${USERNAME} moveit2_tutorials.repos /tmp/
RUN vcs import src < /tmp/moveit2_tutorials.repos

# Update the ownership of the source files (had to use sudo above to work around
# a possible inherited 'insteadof' from the host that forces use of ssh
RUN sudo chown -R ${USERNAME}:${USERNAME} ${MOVEIT2_DIR}

RUN touch DEBUG-02.txt

# Install system dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash' \
  && rosdep install --from-paths ../spaceros/src src --ignore-src \
     --rosdistro ${ROSDISTRO} -r -y --skip-keys \
     "console_bridge generate_parameter_library fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers rmw_connextdds ros_testing rmw_connextdds rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp composition demo_nodes_py lifecycle rosidl_typesupport_fastrtps_cpp rosidl_typesupport_fastrtps_c ikos diagnostic_aggregator diagnostic_updater joy qt_gui rqt_gui rqt_gui_py"

# Apply a patch to octomap_msgs to work around a build issue
COPY --chown=${USERNAME}:${USERNAME} octomap_fix.diff ./src/octomap_msgs
RUN cd src/octomap_msgs && git apply octomap_fix.diff

# Build MoveIt2
RUN /bin/bash -c 'source ${SPACEROS_DIR}/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers desktop_notification- status-'

# # Add a couple sample GUI apps for testing
# RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
#   --mount=type=cache,target=/var/lib/apt,sharing=locked \
#   sudo apt-get install -y \
#   firefox \
#   glmark2 \
#   libcanberra-gtk3-0 \
#   libpci-dev \
#   xauth \
#   xterm

# === END Adaptation from space-ros/docker/moveit2/Dockerfile ===

RUN touch DEBUG-11.txt

# TODO(anyone): remove demo-pkgs.txt, no packages left after exclusions
# Generate repos file for demo dependencies, excluding packages from Space ROS core.
# COPY --chown=${USERNAME}:${USERNAME} demo-pkgs.txt /tmp/
# COPY --chown=${USERNAME}:${USERNAME} excluded-pkgs.txt /tmp/
# RUN rosinstall_generator \
#   --rosdistro ${ROSDISTRO} \
#   --deps \
#   --exclude-path ${SPACEROS_DIR}/src \
#   --exclude-path ${MOVEIT2_DIR}/src \
#   --exclude $(cat /tmp/excluded-pkgs.txt) -- \
#   -- $(cat /tmp/demo-pkgs.txt) \
#   > /tmp/demo_generated_pkgs.repos

RUN mkdir -p ${DEMO_DIR}/src
WORKDIR ${DEMO_DIR}

RUN touch DEBUG-12.txt

# Install libmongoc for development
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get install libmongoc-dev -y

# Compile mongo cxx driver https://mongocxx.org/mongocxx-v3/installation/linux/
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get install libssl-dev build-essential devscripts debian-keyring fakeroot \
       debhelper cmake libboost-dev libsasl2-dev libicu-dev libzstd-dev doxygen -y
RUN wget https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.6.7/mongo-cxx-driver-r3.6.7.tar.gz
RUN tar -xzf mongo-cxx-driver-r3.6.7.tar.gz
RUN cd mongo-cxx-driver-r3.6.7/build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
  && sudo cmake --build . --target EP_mnmlstc_core \
  && cmake --build . \
  && sudo cmake --build . --target install

RUN touch DEBUG-15.txt

# Get the source for the dependencies
# RUN vcs import src < /tmp/demo_generated_pkgs.repos
COPY --chown=${USERNAME}:${USERNAME} demo_manual_pkgs.repos /tmp/
RUN vcs import src < /tmp/demo_manual_pkgs.repos \
  && /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' 

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt-get update -y \
  && /bin/bash -c 'source "${SPACEROS_DIR}/install/setup.bash"' \
  && /bin/bash -c 'source "${MOVEIT2_DIR}/install/setup.bash"' \
  && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROSDISTRO}

RUN vcs import src < "${DEMO_DIR}/src/mini_pupper_ros/.minipupper.repos"

# HACK: Move out package whose build still fails
RUN /bin/mkdir -p ../NO_src \
  && for d in champ ros2_controllers; do mv "src/$d" "../NO_src/"; done

# Build the demo
RUN /bin/bash -c '\
  source "${SPACEROS_DIR}/install/setup.bash" \
  && source "${MOVEIT2_DIR}/install/setup.bash" \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# === TODO: The command above fails ============================================
# --- stderr: forward_command_controller                            
# CMake Error at CMakeLists.txt:26 (find_package):
#   By not providing "Findcontroller_interface.cmake" in CMAKE_MODULE_PATH this
#   project has asked CMake to find a package configuration file provided by
#   "controller_interface", but CMake did not find one.
# 
#   Could not find a package configuration file provided by
#   "controller_interface" with any of the following names:
# 
#     controller_interfaceConfig.cmake
#     controller_interface-config.cmake
# 
#   Add the installation prefix of "controller_interface" to CMAKE_PREFIX_PATH
#   or set "controller_interface_DIR" to a directory containing one of the
#   above files.  If "controller_interface" provides a separate development
#   package or SDK, be sure it has been installed.
# 
# 
# ---
# Failed   <<< forward_command_controller [1.84s, exited with code 1]
# ==============================================================================

RUN touch DEBUG-21.txt

# # Add the user to the render group so that the user can access /dev/dri/renderD128
# RUN sudo usermod -aG render $USERNAME

# # Setup the entrypoint
# COPY ./entrypoint.sh /
# ENTRYPOINT ["/entrypoint.sh"]
# CMD ["bash"]

# EOF
