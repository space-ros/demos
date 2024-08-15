FROM osrf/space-ros:core

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install dependencies
RUN sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Prepare the workspace
SHELL ["bash", "-c"]
RUN mkdir -p ${HOME}/canadarm_ws/src
WORKDIR ${HOME}/canadarm_ws

COPY ./canadarm_demo src/canadarm_demo
COPY .defaults.yaml .defaults.yaml

# Build the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /opt/spaceros/setup.bash \
    && colcon build

# Source the workspace
RUN echo "source ${HOME}/canadarm_ws/install/setup.bash" >> ${HOME}/.bashrc
