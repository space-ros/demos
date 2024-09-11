FROM openrobotics/space_robots_demo AS base

# Define arguments used in the metadata definition
ARG VCS_REF
ARG VERSION="preview"

# Specify the docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="Isaac-sim demo"
LABEL org.label-schema.description="Preview version of the Isaac-sim/Space ROS platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://github.com/space-ros"
LABEL org.label-schema.vcs-url="https://github.com/space-ros/docker"
LABEL org.label-schema.vcs-ref=${VCS_REF}

FROM base

ENV DEBIAN_FRONTEND=noninteractive

USER root

COPY --link apt-requirements.txt /apt-requirements.txt
# hadolint ignore=SC2046
RUN apt-get update && \
    apt-get install --no-install-recommends -y $(cat /apt-requirements.txt) && \
    rm -rf /var/lib/apt/lists/*
RUN rm /apt-requirements.txt

COPY --link ros-requirements.txt /ros-requirements.txt
# hadolint ignore=SC2046
RUN apt-get update && \
    apt-get install --no-install-recommends -y $(cat /ros-requirements.txt) && \
    rm -rf /var/lib/apt/lists/* && \
    rm /ros-requirements.txt

RUN locale-gen en_US en_US.UTF-8 &&  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# Define workspace locations
ENV NAVIGATION2_WS=${HOME_DIR}/nav2_ws
ENV NAV2_DEPS_WS=${HOME_DIR}/nav2_deps_ws

# Install Isaac Sim
COPY --link nvidia-pip-requirements.txt /requirements.txt
RUN pip install --timeout=600 --no-cache-dir -r /requirements.txt --extra-index-url https://pypi.nvidia.com && \
    rm /requirements.txt

RUN adduser $USERNAME sudo && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

RUN mkdir -p /usr/local/lib/python3.10/dist-packages/isaacsim/extscache/omni.gpu_foundation/cache && \
    mkdir -p /usr/local/lib/python3.10/dist-packages/omni/logs && \
    mkdir -p /usr/local/lib/python3.10/dist-packages/omni/cache/Kit/106.0/fcefe91f/PhysXLocalMeshCache && \
    mkdir -p /usr/local/lib/python3.10/dist-packages/omni/data/documents/Kit/apps/Isaac-Sim/scripts/new_stage && \
    mkdir -p /usr/local/lib/python3.10/dist-packages/omni/data/documents/Kit/shared/screenshots && \
    chown -R $USERNAME:$USERNAME /usr/local/lib/python3.10/dist-packages/isaacsim/extscache/omni.gpu_foundation/cache && \
    chown -R $USERNAME:$USERNAME /usr/local/lib/python3.10/dist-packages/omni/logs && \
    chown -R $USERNAME:$USERNAME /usr/local/lib/python3.10/dist-packages/omni/cache && \
    chown -R $USERNAME:$USERNAME /usr/local/lib/python3.10/dist-packages/omni/data/documents && \
    chmod -R 777 /usr/local/lib/python3.10/dist-packages/isaacsim/* && \
    chmod -R 777 /usr/local/lib/python3.10/dist-packages/omni/logs && \
    chmod -R 777 /usr/local/lib/python3.10/dist-packages/omni/cache && \
    chmod -R 777 /usr/local/lib/python3.10/dist-packages/omni/data/documents

USER ${USERNAME}

# Source ROS2 setup.bash and build your workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    mkdir -p /home/$USERNAME/isaac_ws/src && \
    cd /home/$USERNAME/isaac_ws/src && \
    git clone --recurse-submodules https://github.com/AlexanderRex/isaac_ros2_utils.git && \
    cd isaac_ros2_utils && git submodule update --init --recursive && \
    cd /home/$USERNAME/isaac_ws/src && \
    git clone https://github.com/AlexanderRex/curiosity_rover_description.git && \
    git clone https://github.com/AlexanderRex/mars_rover_control.git && \
    cd /home/$USERNAME/isaac_ws && colcon build"

# Set environment in bashrc for future sessions
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc && \
    echo 'source /home/$USERNAME/isaac_ws/install/setup.bash' >> /home/$USERNAME/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /home/$USERNAME/.bashrc

COPY --link entrypoint.sh /.entrypoint.sh
ENTRYPOINT ["/.entrypoint.sh"]
