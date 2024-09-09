# Space ROS Demos

This repository provides examples for running robots using Space ROS.

Submissions to this repo should include:
1) A Dockerfile or docker-compose.yaml file for building on top of the `osrf/space-ros:latest`
2) A `build.sh` script for building the docker image
    - This is required for CI to build the image
    - See example here: https://github.com/space-ros/docker/blob/main/space_robots/build.sh
3) A `run.sh` script for running the demo
    - - See example here: https://github.com/space-ros/docker/blob/main/space_robots/run.sh

Please refer to the [dockerfile repo](https://github.com/space-ros/docker/tree/main/space_robots) for instructions on running the existing demos.

To run the examples that are a part of the `spaceros_gz_demos` package, refer to the README [here](spaceros_gz_demos/README.md).
