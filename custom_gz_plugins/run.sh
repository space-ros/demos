#!/bin/bash

DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

xhost +local:root

ORG=spaceros
IMAGE=custom_sim_custom_plugins_demo
TAG=latest

container_name="space_ros_custom_gz_plugins_demo"

ros_domain_id=$(printenv ROS_DOMAIN_ID)

# Check if a Docker container with "$container_name" is already running.
# If it is running, open an interactive bash shell inside it.
# If the container is not running, create and run a new Docker container with specified configurations
if docker ps --format '{{.Names}}' | grep -q "$container_name"; then
    docker exec -it "$container_name" /bin/bash
else
    docker run -it --rm \
        ${DOCKER_ARGS[@]} \
        -e DISPLAY=$DISPLAY \
        -v $PWD/..:/workspaces/sim_ws/src \
        -v /var/run/docker.sock:/var/run/docker.sock \
        --name "$container_name" \
        --workdir /home/spaceros-user/demos_ws/src \
        --env ROS_DOMAIN_ID=$ros_domain_id \
        --runtime nvidia \
        --network host \
        -v /dev/input:/dev/input --device-cgroup-rule='c 13:* rmw' \
        $@ \
        $ORG/$IMAGE:$TAG \
        /bin/bash
fi
