#!/usr/bin/env bash

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   an X server

IMG_NAME=baroloteam/freisa_on_mars

# Replace `/` with `_` to comply with docker container naming
# And append `_runtime`
CONTAINER_NAME="$(tr '/' '_' <<< "$IMG_NAME")"

# Sanity checks
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
    echo "Warning: DISPLAY undefined, set to $DISPLAY"
fi
if [ -z "$TERM" ]; then
    export TERM=xterm
    echo "Warning: TERM undefined, set to $TERM"
fi
if [ -z "$XAUTHORITY" ]; then
    export XAUTHORITY=~/.Xauthority
    echo "Warning: XAUTHORITY undefined, set to $XAUTHORITY"
    xauth extract \- $DISPLAY >>$XAUTHORITY
fi

# Start the container
#
# --rm: delete container after exiting
# -it: Interactive TTY mode so you can use a shell.
# --network host: Does not isolate Docker container from host's network interfaces.
#                 Allows all traffic to pass through as if it originated from the host.
#                 In our case, useful for DDS traffic for ROS.
# -e DISPLAY: Pass the X11 display through to the container.
# -e TERM: Pass the kind of terminal being used through to the container.
# -e QT_X11_NO_MITSHM=1: Disables shared memory extension for Qt/X11, which does not work consistently inside a process namespace like a container.
# -v "$XAUTHORITY:/.Xauthority": the $XAUTHORITY environment variable on the host contains the path to a file that grants access to X11.
#                                We mount the path defined in that environment variable into the container at /.Xauthority.
# -e XAUTHORITY=/.Xauthority: We set $XAUTHORITY *inside* the container to the place we just mounted the Xauthority (/.Xauthority)
#
# Credits:
# * https://github.com/space-ros/docker/pull/168
# * https://github.com/space-ros/docker/pull/175
#
docker run --rm -it --name $CONTAINER_NAME \
    --network host \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --volume "$XAUTHORITY:/.Xauthority" \
    --device=/dev/dri:/dev/dri \
    -e DISPLAY \
    -e TERM \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=/.Xauthority \
    $IMG_NAME


# EOF
