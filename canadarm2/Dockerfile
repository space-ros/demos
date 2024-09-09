FROM osrf/space-ros:latest

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DISTRO=humble
ENV HOME=/home/spaceros-user

# Install dependencies
# Related to space-ros/space-ros#195 and space-ros/space-ros#196
RUN sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Prepare the workspace
SHELL ["bash", "-c"]
RUN mkdir -p ${HOME}/canadarm_ws/src
WORKDIR ${HOME}/canadarm_ws

COPY ./canadarm_demo src/canadarm_demo
COPY ./canadarm_common src/canadarm_common
COPY .defaults.yaml .defaults.yaml

# Build the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source $HOME/spaceros/install/setup.bash \
    && colcon build

# Source the workspace
RUN echo "source ${HOME}/canadarm_ws/install/setup.bash" >> ${HOME}/.bashrc
