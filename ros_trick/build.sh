#!/bin/bash

set -e

TAG=latest
ROS_TRICK_BRIDGE_IMAGE_NAME=ros_trick_bridge
CANADARM_ROS_TRICK_DEMO_NAME=canadarm_ros_trick_demo
export SPACE_ROS_IMAGE=osrf/space-ros:jazzy-2025.04.0
CURRENT_PATH=$(pwd)
SPACEROS_DOCKER_REPO_PATH="${CURRENT_PATH}/docker_repo"

if [ ! -d ${SPACEROS_DOCKER_REPO_PATH} ]; then
    mkdir -p ${SPACEROS_DOCKER_REPO_PATH}
    git clone https://github.com/space-ros/docker.git ${SPACEROS_DOCKER_REPO_PATH}
fi
echo "*** Building moveit2 ***"
cd ${SPACEROS_DOCKER_REPO_PATH}/moveit2 && ./build.sh
echo "*** Building space_robots ***"
cd ${SPACEROS_DOCKER_REPO_PATH}/space_robots && ./build.sh
cd ${CURRENT_PATH}

echo "*** Building ros_trick_bridge.Dockerfile ***"
docker build -t ${ROS_TRICK_BRIDGE_IMAGE_NAME}:${TAG} -f ./ros_trick_bridge.Dockerfile .
echo "*** Building canadarm_ros_trick_demo.Dockerfile ***"
docker build -t ${CANADARM_ROS_TRICK_DEMO_NAME}:${TAG} -f ./canadarm_ros_trick_demo.Dockerfile .
