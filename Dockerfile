
FROM ros:humble

# Define a few key variables
ENV USERNAME=spaceros-user
ENV HOME_DIR=/home/spaceros-user
ENV DEMO_DIR=${HOME_DIR}/demo_ws
ENV ROSDISTRO=humble


# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Get rosinstall_generator
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
RUN sudo apt update && sudo apt install -y python3-pip \
    ros-${ROSDISTRO}-control-msgs \
    ros-${ROSDISTRO}-rviz-common \
    ros-${ROSDISTRO}-rmw-cyclonedds-cpp \
    ros-${ROSDISTRO}-geometry-msgs \
    ros-${ROSDISTRO}-sensor-msgs \
    ros-${ROSDISTRO}-std-msgs \
    ros-${ROSDISTRO}-rosidl-typesupport-fastrtps-cpp \
    ros-${ROSDISTRO}-rosidl-typesupport-fastrtps-c \
    ros-${ROSDISTRO}-rviz2 \
    && sudo rm -rf /var/lib/apt/lists/*

RUN pip install vcstool
RUN mkdir -p ${DEMO_DIR}/src
WORKDIR ${DEMO_DIR}


# Get the source for the dependencies
COPY --chown=${USERNAME}:${USERNAME} demo_manual_pkgs.repos /tmp/
RUN vcs import src < /tmp/demo_manual_pkgs.repos

#RUN sudo apt-get update -y && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROSDISTRO}


# Build the demo
RUN /bin/bash -c 'source /opt/ros/${ROSDISTRO}/setup.bash \
     && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev'

# Setup the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
