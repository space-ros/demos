FROM osrf/space-ros:latest

# Install Gazebo Harmonic:
# https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu
RUN sudo apt-get update
RUN sudo apt-get install lsb-release gnupg
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update

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
        gz-harmonic \
        ros-humble-rviz2 \
        ros-humble-ros-gzharmonic \
    && sudo apt-get autoremove -y 

# Replace old entrypoint with an entrypoint that extends upon it
# This is necessary as Gazebo Harmonic has been installed as well as some new ROS packages.
RUN sudo mv /ros_entrypoint.sh /old_entrypoint.sh
COPY entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh

# Set up workspace
RUN mkdir -p /home/spaceros-user/spaceros/ws/src/spaceros_gz_demos
WORKDIR /home/spaceros-user/spaceros/ws
# Copy over the files so that we can pre-build the package with colcon
COPY spaceros_gz_demos /home/spaceros-user/spaceros/ws/src/spaceros_gz_demos
# Source ROS environment and build the package
RUN . /opt/ros/humble/setup.sh && . ${SPACEROS_DIR}/install/setup.sh && colcon build
