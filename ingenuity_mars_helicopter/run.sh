#!/usr/bin/env bash

# Runs a docker container with the image created by build.sh
# Requires:
#   docker
#   an X server

IMG_NAME=ingenuity_flight_simulator

# Replace `/` with `_` to comply with docker container naming
# And append `_runtime`
CONTAINER_NAME="$(tr '/' '_' <<< "$IMG_NAME")"


# Get the directory of the current script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Go one level up
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

# Start the container
# if you don't have a graphic nvidia card, remove the --gpus all flag
docker run --rm -it --gpus all \
  --name "$CONTAINER_NAME" \
  --network host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  -e DISPLAY="$DISPLAY" \
  -e TERM="$TERM" \
  -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  "$IMG_NAME"