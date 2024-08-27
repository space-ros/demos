# LunarSim Demo
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)\
This folder containts three ROS2 packages for to enable simulation in a lunar environment, including a lunar gazebo world using a Digital Elevation Model (DEM) and a plugin for a dynamic sun model based on Ephemeris data.

Instructions for building this package can be found in the space-ros-docker repo.

## lunar_sun_gz_plugin
This package contains a gazebo plugin to move an actor and create a light source at the location of the actor. 
The plugin must be added to an actor named `animated_sun`, which can be done as follows:
```
<actor name="animated_sun">
    <plugin name="LunarSun" filename="liblunar_sun_gz_plugin.so"/>
    Other tags ...
</actor>
```
The suns trajectory is based on the `horizons_az_el.csv` file. Detailed documentation on updating this can be found on the space-ros lunarsim docs page.

## lunarsim_gz_worlds
This package contains the lunarsim world files, including the world sdf, DEM files, textures and tools for creating textures. It also contains the `display.launch.py` launch file, which launches gazebo with the lenarsim world by default (but does not spawn the rover). For detailed documentation on updating DEM model and textures see the space-ros lunarsim docs page.

## lunarsim_gz_bringup
This package contains launch and configuration files to launch the lunarsim world and spawn [leo_rover](https://github.com/LeoRover) with keyboard controls. The `spawn_leo_robot.launch.py` spawns the leo model, starts`gz_bridge` and `key_teleop`. The `gz_bridge` parameters can be modified in the `leo_ros_gz_bridge.yaml` file in the config directory.

Challenge Name: NASA Space ROS Sim Summer Sprint Challenge \
Team Lead Freelancer Username: elementrobotics \
Submission Title: LunarSim