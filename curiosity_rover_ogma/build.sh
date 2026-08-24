#!/usr/bin/env bash

echo ""
echo "##### Building Space ROS Demo Docker Image #####"
echo ""

# Build dependency

ORG=nasa
IMAGE=ogma
TAG=latest

VCS_REF=""
VERSION=preview

# Exit script with failure if build fails
set -eo pipefail

# Build the image that contains Ogma
docker build \
    --target builder \
    -t $ORG/$IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" .

# Run Ogma on this project and collect the output
docker build --output type=local,dest=./monitor .

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

VCS_REF=""
VERSION=preview

docker build -t $ORG/$IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" monitor/

echo ""
echo "##### Done! #####"
