#!/bin/bash

# Define variables
APP1="jaxa_spaceros_cfs"
APP2="jaxa_spaceros_listener"

# Define Docker image names
IMAGE1="jaxa_spaceros_cfs"
IMAGE2="jaxa_spaceros_listener"

# Define Dockerfile names
DOCKERFILE1="Dockerfile.${APP1}"
DOCKERFILE2="Dockerfile.${APP2}"

# Exit script with failure if build fails
set -eo pipefail

# Build Docker images
echo "Building Docker image for ${APP1}..."
docker build -t $IMAGE1 -f $DOCKERFILE1 .

echo "Building Docker image for ${APP2}..."
docker build -t $IMAGE2 -f $DOCKERFILE2 .

echo "Docker images built successfully."

