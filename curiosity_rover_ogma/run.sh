#!/usr/bin/env bash

# Runs a docker container with the image created by build.bash
# Requires:
#   docker

IMG_NAME=osrf/space-ros-curiosity-rover-ogma
IMG_TAG=latest

# Replace `/` with `_` to comply with docker container naming
CONTAINER_NAME="$(tr '/' '_' <<< "$IMG_NAME")"

# Start the container
docker run \
  --rm \
  -it \
  --name "${CONTAINER_NAME}" \
  --network host \
  -e TERM \
  "${IMG_NAME}:${IMG_TAG}" \
  /bin/bash
