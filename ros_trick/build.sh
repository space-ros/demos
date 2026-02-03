#!/bin/bash

set -e

TAG=latest
ROS_TRICK_BRIDGE_IMAGE_NAME=ros_trick_bridge
CANADARM_ROS_TRICK_DEMO_NAME=canadarm_ros_trick_demo
export SPACE_ROS_IMAGE=osrf/space-ros:main
CURRENT_PATH=$(pwd)

echo "*** Building space_robots ***"
cd ../space_robots && ./build.sh
cd ${CURRENT_PATH}

echo "*** Building ros_trick_bridge.Dockerfile ***"
docker build -t ${ROS_TRICK_BRIDGE_IMAGE_NAME}:${TAG} -f ./ros_trick_bridge.Dockerfile .
echo "*** Building canadarm_ros_trick_demo.Dockerfile ***"
docker build -t ${CANADARM_ROS_TRICK_DEMO_NAME}:${TAG} -f ./canadarm_ros_trick_demo.Dockerfile .
