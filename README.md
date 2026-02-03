# Space ROS Demos

This repository provides examples for running robots using Space ROS

Submissions to this repo should include:
1) A Dockerfile or docker-compose.yaml file for building on top of the `osrf/space-ros:latest`
2) A `build.sh` script for building the docker image
    - This is required for CI to build the image
3) A `run.sh` script for running the demo

## Demos

1. [Canadarm2](canadarm2/README.md)
2. [Curiosity Rover](curiosity_rover/README.md)
3. [Nav2 Demo](nav2_demo/README.md)
4. [ROS Trick Demo](ros_trick/README.md)
5. [Space Robots Demo](space_robots/README.md)

