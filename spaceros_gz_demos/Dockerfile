FROM osrf/space-ros:latest

# Install required packages
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        bash \
        bash-completion \
        build-essential \
        cmake \
        git \
        python-is-python3 \
        python3-pip \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-rviz2 \
    && sudo apt-get autoremove -y 

# Replace old entrypoint with an entrypoint that extends upon it
# This is necessary as Gazebo Harmonic has been installed as well as some new ROS packages.
RUN sudo rm /entrypoint.sh
COPY entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

# Set up workspace
RUN mkdir -p /home/spaceros-user/spaceros/ws/src/spaceros_gz_demos
WORKDIR /home/spaceros-user/spaceros/ws
# Copy over the files so that we can pre-build the package with colcon
COPY spaceros_gz_demos /home/spaceros-user/spaceros/ws/src/spaceros_gz_demos
# Source ROS environment and build the package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && . ${SPACEROS_DIR}/setup.sh && colcon build
