#!/usr/bin/env bash

sudo xhost +

declare SCRIPT_NAME=$(readlink -f ${BASH_SOURCE[0]})
cd $(dirname $SCRIPT_NAME)

docker compose run --name space_ros_isaac_sim --rm space_ros_isaac_sim --remove-orphans
