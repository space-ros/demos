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

docker build -t $ORG/$IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" \
    $DIR  # Dockerfile is in the current directory, no need for -f

echo ""
echo "##### Done! #####"
