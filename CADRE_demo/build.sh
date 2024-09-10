#!/usr/bin/env bash

ORG=openrobotics
IMAGE=CADRE_demo
TAG=latest

VCS_REF=""
VERSION=preview

# Exit script with failure if build fails
set -eo pipefail

echo ""
echo "##### Building Space ROS Demo Docker Image #####"
echo ""

# get the folder of the script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DOKCERDIR="$DIR/Dockerfile"

docker build -t $ORG/$IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" $DIR \
    -f $DOKCERDIR

echo ""
echo "##### Done! #####"

