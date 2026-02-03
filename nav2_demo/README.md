# This folder contains the necessary files for running Nav2 on the Mars Rover demo

This demo is a preliminary release, please file any issues found.

First build this container using the build script

```bash
./build.sh
```

There will now be a docker image called "osrf/space-ros-nav2-demo:latest".

## Run the Mars Rover demo with Nav2 and SLAM toolbox

## Terminal 1 - launch the mars_rover demo

Follow the instructions to build and run the mars rover demo in [space-robots/README.md](../space_robots/README.md).

## Terminal 2 - launch Nav2

Start the space-ros-nav2 container and launch the navigation2 nodes:

```
./run.sh
ros2 launch space_ros_nav2_bringup navigation_launch.py use_sim_time:=True params_file:=nav2_params.yaml
```

## Terminal 3 - launch localization with map

```
docker exec -it osrf_space-ros-nav2-demo bash
source install/setup.bash
ros2 launch space_ros_nav2_bringup localization_launch.py use_sim_time:=True map:=mars_map.yaml params_file:=nav2_params.yaml
```

## Terminal 4 - launch Rviz

Exec into the same space-ros-nav2 container and launch Rviz2:

```
docker exec -it -e DISPLAY osrf_space-ros-nav2-demo bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nav2_bringup rviz_launch.py
```

## Localize the robot and send a Nav2 goal

In the Rviz GUI, localize robot in Rviz using 2D Pose estimate tool, with an arrow pointing up at the center of the grid.
Wait for Nav2 to initialize / costmaps to appear.
When the costmap appears, send a goal using the "Nav2 goal" tool in Rviz
