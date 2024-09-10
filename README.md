# Space ROS Demos

This repository provides examples for running robots using Space ROS

Submissions to this repo should include:
1) A Dockerfile or docker-compose.yaml file for building on top of the `osrf/space-ros:latest`
2) A `build.sh` script for building the docker image
    - This is required for CI to build the image
    - See example here: https://github.com/space-ros/docker/blob/main/space_robots/build.sh
3) A `run.sh` script for running the demo
    - - See example here: https://github.com/space-ros/docker/blob/main/space_robots/run.sh

Please refer to the [dockerfile repo](https://github.com/space-ros/docker/tree/main/space_robots) for instructions on running the existing demos

______________________________

## Changes made to Repository
See ReadME in `spot_ros2_ign` package. The `spot_ros2_ign` submodule was added, an adaption of champ quadruped robot to ROS2 and Ignition Gazebo. Previously, the Boston Dynamics Spot robot was only simulated using ROS2 with Gazebo Classic, which goes end-of-life in January 2025. This package, modified from quadruped_ros2, simulates the Spot robot in Ignition Gazebo, a newer version of Gazebo. This combination of simulating the Spot robot with ROS2 and Ignition Gazebo on an open-source platform has not been done yet. This package also publishes an Rviz panel where the robot is visible and able to be controlled with its joints.
