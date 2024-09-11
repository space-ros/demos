#!/usr/bin/env bash

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   an X server

IMG_NAME=openrobotics/o3de_curiosity_docker

# Replace `/` with `_` to comply with docker container naming
# And append `_runtime`
CONTAINER_NAME="$(tr '/' '_' <<< "$IMG_NAME")"

# TODO this needs to be updated with 
# Start the container
docker run --rm -it --name $CONTAINER_NAME  --network host \
    -e DISPLAY -e TERM   -e QT_X11_NO_MITSHM=1 $IMG_NAME

# xhost +local:docker

# docker run -d \
# --name=o3de_rosbot_xl \
# --runtime=nvidia \
# -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
# -e DISPLAY=${DISPLAY} \
# -e NVIDIA_VISIBLE_DEVICES=all \
# -e NVIDIA_DRIVER_CAPABILITIES=all \
# o3de-husarion /data/workspace/ROSbotXLDemo/build/linux/bin/profile/Editor

