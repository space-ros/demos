#!/bin/bash
echo "Building docker image..."
docker build . --tag "spaceros_gz_demos"
