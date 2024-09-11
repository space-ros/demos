# Mars Rover Teleop RViz2 Plugin

1. This package provides an rviz2 plugin for tele-operation of the curiosity rover with **mars_rover_nvidia_isaac package**
2. Config file **config > rviz_with_teleop.rviz** loads the plugin at bottom left rocation in RViz2.
   ```bash
   rviz2 -d <path/to/rviz_with_teleop.rviz>
   ```
3. Config file **config > rviz_teleop_sensors.rviz** loads RViz2 with the plugin as well as sensor data.
   ```bash
   rviz2 -d <path/to/rviz_teleop_sensors.rviz>
   ```