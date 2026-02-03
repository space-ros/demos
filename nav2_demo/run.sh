#!/usr/bin/env bash

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   an X server

IMG_NAME=osrf/space-ros-nav2-demo
IMG_TAG=latest

# Replace `/` with `_` to comply with docker container naming
CONTAINER_NAME="$(tr '/' '_' <<< "$IMG_NAME")"

# Start the container
docker run \
  --rm \
  -it \
  --name "${CONTAINER_NAME}" \
  --network host \
  -e DISPLAY \
  -e TERM \
  -e QT_X11_NO_MITSHM=1 \
  "${IMG_NAME}:${IMG_TAG}"

