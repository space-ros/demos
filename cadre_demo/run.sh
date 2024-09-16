#!/usr/bin/env bash

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   an X server (for GUI applications)

IMG_NAME=openrobotics/cadre_demo

# Replace `/` with `_` to comply with docker container naming
# And append `_runtime`
CONTAINER_NAME="$(tr '/' '_' <<< "$IMG_NAME")"

# Start the container with an interactive bash shell
docker run --rm -it --name $CONTAINER_NAME --network host \
    -e DISPLAY=$DISPLAY \
    -e TERM=xterm \
    -e QT_X11_NO_MITSHM=1 \
    $IMG_NAME /bin/bash -c "
    echo 'Starting container...';
    echo 'ROS 2 environment setup:';
    source /opt/ros/humble/setup.bash;
    echo 'ROS 2 environment variables:';
    printenv | grep ROS;
    echo 'Checking ROS 2 version:';
    ros2 --version;
    echo 'Listing ROS 2 packages:';
    ros2 pkg list;
    echo 'Current directory:';
    pwd;
    echo 'Contents of /root/cadre_demo/src:';
    ls -la /root/cadre_demo/src;
    echo 'Launching interactive shell...';
    exec /bin/bash"
