#!/usr/bin/env bash

ORG=openrobotics
IMAGE=mars_helicopter
TAG=latest

VCS_REF=""
VERSION=preview

# Exit script with failure if build fails
set -eo pipefail

echo ""
echo "##### Building Space ROS Demo Docker Image #####"
echo ""

docker build --build-arg CACHEBUST=$(date +%s) -t $ORG/$IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" . \

echo ""
echo "##### Done! #####"