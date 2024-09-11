#!/usr/bin/env bash

sudo xhost +

declare SCRIPT_NAME=$(readlink -f ${BASH_SOURCE[0]})
cd $(dirname $SCRIPT_NAME)

docker compose build space_ros_isaac_sim