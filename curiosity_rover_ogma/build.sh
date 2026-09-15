#!/usr/bin/env bash

echo ""
echo "##### Building Space ROS Demo Docker Image #####"
echo ""

# Build dependency

ORG=nasa
IMAGE=ogma
TAG=latest

# Exit script with failure if build fails
set -eo pipefail

# Run Ogma on this project and collect the output
docker build -t $ORG/$IMAGE:$TAG --output type=local,dest=./monitor .

# Build dependency used for this demo
if ! docker image inspect "osrf/space-ros:curiosity_demo" >/dev/null 2>&1; then
  pushd ../curiosity_rover/
  ./build.sh
  popd
fi

# Build generated app itself
ORG=osrf
IMAGE=space-ros-curiosity-rover-ogma
TAG=latest

docker build -t $ORG/$IMAGE:$TAG monitor/

echo ""
echo "##### Done! #####"
