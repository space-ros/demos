# Base image
FROM osrf/space-ros:latest

# Switch to root user
USER root

# Update and install base packages
RUN apt-get update && apt-get dist-upgrade -y
RUN apt-get install -y \
  build-essential \
  clang-format \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget \
  # Install additional utilities
  ros-humble-xacro \
  ros-humble-rviz2 \
  # Clean up to reduce image size
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-ros2launch \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install colcon extensions
RUN pip install colcon-common-extensions

# Set the working directory
WORKDIR /app

# Create the workspace directory for ROS 2
RUN mkdir -p /root/cadre_demo/src

# Copy project files into the workspace
COPY . /root/cadre_demo/src/

# Set environment variables
ENV ROS_WORKSPACE=/root/cadre_demo
ENV ROS_DISTRO=humble

# Source ROS 2 setup script in the container environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Build the ROS 2 workspace
RUN bash -c "source /opt/ros/humble/setup.bash \
    && cd /root/cadre_demo \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install"

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec /root/cadre_demo/src/entrypoint.sh"]
