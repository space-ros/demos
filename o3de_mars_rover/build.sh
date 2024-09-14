#!/usr/bin/env bash

# This shell script will build openrobotics/moveit2, openrobotics/space_robot_demo and then

ORG=openrobotics
IMAGE1=moveit2
IMAGE2=space_robot_demo
IMAGE3=o3de_curiosity_docker
TAG=latest

VCS_REF=""
VERSION=preview

# Directory where Dockerfiles are stored
DOCKER_DIR="./docker"

# Exit script with failure if build fails
set -eo pipefail


echo ""
echo "##### Building moviet2 image #####"

docker build -t $ORG/$IMAGE1:$TAG \
    -f $DOCKER_DIR/Dockerfile_moveit2 \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" .

echo ""
echo "##### Done! #####"

# echo ""
# echo "##### Building O3DE + Space ROS Demo Docker Image #####"
# echo ""

# docker build -t $ORG/$IMAGE:$TAG \
#     --build-arg VCS_REF="$VCS_REF" \
#     --build-arg VERSION="$VERSION" .

# echo ""
# echo "##### Done! #####"

