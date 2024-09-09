#!/bin/bash

# shellcheck disable=SC2089
XTERM_CONFIG="-bg black -fg white -fa 'Monospace' -fs 11"
DOCKER_IMAGE="osrf/space-ros:canadarm_demo"
LOCAL_WORKSPACE=$(pwd)

# Run the CanadArm Gazebo simulation locally
run-gazebo() {
    # shellcheck disable=SC2090
    # shellcheck disable=SC2086
    xterm $XTERM_CONFIG -T 'CanadArm2 Gazebo' -e "source /opt/ros/\${ROS_DISTRO}/setup.bash \
        && source $LOCAL_WORKSPACE/install/setup.bash \
        && ros2 launch canadarm_gazebo canadarm.launch.py" &
}

run-control-demo() {
    # shellcheck disable=SC2090
    # shellcheck disable=SC2086
    xterm $XTERM_CONFIG -T 'CanadArm2 Demo' -e "docker run -it --rm \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        $DOCKER_IMAGE \
        bash -c 'source /home/spaceros-user/canadarm_ws/install/setup.bash && ros2 launch canadarm_demo canadarm.launch.py'" &
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
