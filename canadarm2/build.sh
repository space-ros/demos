#!/bin/bash

set -e

# Define variables
DOCKER_IMAGE="osrf/space-ros:canadarm_demo"
LOCAL_WORKSPACE=$(pwd)
SHELL="/bin/bash"

# Define packages
SIM_PACKAGES="canadarm_description canadarm_gazebo"
CONTROL_PACKAGE="canadarm_moveit_config canadarm_demo"
DEMO_PACKAGES="$CONTROL_PACKAGE $SIM_PACKAGES"

####################################################################################################
# High-level functions
####################################################################################################
# Help function to describe each target
help() {
    echo "CanadArm2 Build Script"
    echo "Usage: ./build.sh [subcommand] [demo]"
    echo "Available subcommands:"
    echo "  ./build.sh help                   - Show this help message"
    echo "  ./build.sh clean                - Clean the local workspace"
    echo "  ./build.sh                        - Build the demos"
    exit 0
}

# Build all
build() {
    build-control-demo
}

# Clean the local workspace
clean() {
    rm -rf "$LOCAL_WORKSPACE"/install "$LOCAL_WORKSPACE"/log "$LOCAL_WORKSPACE"/build
}

check_docker() {
    if ! command -v docker &>/dev/null; then
        echo "Docker is not installed. Please install Docker to continue."
        exit 1
    fi
}

check_xterm() {
    if ! command -v xterm &>/dev/null; then
        echo "xterm is not installed. Please install xterm to continue."
        echo "On Ubuntu, you can install xterm with 'sudo apt install xterm'."
        exit 1
    fi
}

check_ros2() {
    if ! command -v ros2 &>/dev/null; then
        echo "ROS 2 is not installed. Please install ROS 2 to continue."
        exit 1
    fi
}

####################################################################################################
# Low-level functions
####################################################################################################
# Build the Docker image
build-docker() {
    docker build -t "$DOCKER_IMAGE" .
}

# Build the Gazebo workspace locally
build-gazebo() {
    # shellcheck source=/opt/ros/${ROS_DISTRO}/setup.bash
    # shellcheck disable=SC1091
    source /opt/ros/"${ROS_DISTRO}"/setup.bash &&
        rosdep install --from-paths . -r -y --skip-keys "$DEMO_PACKAGES" &&
        colcon build --symlink-install --base-paths "$LOCAL_WORKSPACE" --install-base "$LOCAL_WORKSPACE"/install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --packages-select $SIM_PACKAGES
}

####################################################################################################
# Control demo
####################################################################################################
build-control-demo() {
    build-docker
    build-gazebo
}

####################################################################################################
# Main script logic to call the appropriate function
####################################################################################################
case "$1" in
help)
    help
    ;;
clean)
    clean
    ;;
*)
    check_docker
    check_ros2

    build
    ;;
esac
