#!/bin/bash

# shellcheck disable=SC2089
XTERM_CONFIG="-bg black -fg white -fa 'Monospace' -fs 11"
DOCKER_IMAGE="osrf/space-ros:curiosity_demo"
LOCAL_WORKSPACE=$(pwd)

# Run the Curiosity Rover Gazebo simulation locally
run-gazebo() {
    # shellcheck disable=SC2090
    # shellcheck disable=SC2086
    xterm $XTERM_CONFIG -T 'Curiosity Rover Gazebo' -e "source /opt/ros/\${ROS_DISTRO}/setup.bash \
        && source $LOCAL_WORKSPACE/install/setup.bash \
        && ros2 launch curiosity_gazebo curiosity_gazebo.launch.py" &
}

run-control-demo() {
    # shellcheck disable=SC2090
    # shellcheck disable=SC2086
    xterm $XTERM_CONFIG -T 'Curiosity Rover Demo' -e "docker run -it --rm \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        $DOCKER_IMAGE \
        bash -c 'source ~/.bashrc && ros2 launch curiosity_rover_demo mars_rover.launch.py'" &
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

run() {

    check_docker
    check_xterm
    check_ros2

    run-gazebo
    run-control-demo
}

run
