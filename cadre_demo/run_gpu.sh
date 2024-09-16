#!/usr/bin/env bash

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   an X server

IMG_NAME=openrobotics/cadre_demo

xhost +local:docker

# Replace `/` with `_` to comply with docker container naming
# And append `_runtime`
CONTAINER_NAME="$(tr '/' '_' <<< "$IMG_NAME")"

# Start the container
docker run --rm -it --name $CONTAINER_NAME \
    --network host \
    --privileged \
    --gpus all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics \
    -e DISPLAY=$DISPLAY \
    -e TERM \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTHORITY \
    --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
    $IMG_NAME