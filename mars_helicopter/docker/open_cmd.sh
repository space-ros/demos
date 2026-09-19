#!/usr/bin/env bash

# Name of the container
CONTAINER_NAME="openrobotics_mars_helicopter"

# Check if the container is running
if ! docker ps --format '{{.Names}}' | grep -q "$CONTAINER_NAME"; then
  echo "Container $CONTAINER_NAME is not running."
  exit 1
fi

# Connect to the running container and set up the environment
# Assuming `entrypoint.sh` handles setting up the environment inside the container
docker exec -it $CONTAINER_NAME /bin/bash -c "source /home/spaceros-user/demo_ws/src/mars_helicopter/docker/entrypoint.sh && /bin/bash"