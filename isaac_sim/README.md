![space-ros_isaac-sim_front](resources/rover/repo_front.png)
# Space ROS Isaac Sim: Curiosity's Sulfur Stone Discovery in the Gediz Vallis Channel Demo

This repository provides a Docker image for our **NASA's Curiosity rover** demo based on **NVidia Isaac Sim** that interfaces with **Space ROS**. We also built a digital twin of the **Gediz Vallis Channel Environment**, focusing on the area surrounding the recent event of **sulfur stone discovery**. The map was developed using the **HIRISE Mars DTM** (real Mars terrain data) and with custom-made rock assets that seamlessly blend into the environment. It provides a highly detailed replication of the environment for both current and future missions. As of September 2024, Curiosity is still exploring within this region.

See [Map Documentation](./docs/maps.md)

## Highlight of the features
- Docker image of NVIDIA Isaac Sim with Space ROS interface
- Digital twin environment of the Gediz Vallis channel sulfur stone discovery site with enhenced terrain features
- Curiosity rover asset integration into the simulation
- ROS2 interfaces for controlling the rover and sensor feedback

## Advantages of NVidia Isaac Sim
- Realistic graphic with Ray-Tracing technology
- PhysX engine for advanced physics simulation
- Built-in DLSS Frame Generation for improved performence
- Direct VR/AR support (with extension), useful in planetory exploration simulations

## System Requirements

To run the Curiosity rover simulation in Isaac Sim, ensure your system meets the following requirements:

| Requirement  | Description                                  |
| ------------ | -------------------------------------------- |
| OS           | Ubuntu 22.04                                 |
| GPU          | Nvidia RTX 30XX series or higher             |
| RAM          | 16 GB or higher                              |
| CPU          | Intel i7 or higher                           |
| NVIDIA Driver| ver. 560.35.03 (exact ver. recommended)      |

For more detailed hardware requirements, refer to the [Isaac Sim Hardware Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html).

## Prerequisites

The following software dependencies must be installed on your system:

| Dependency         | Description                                                                                                                                 |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------- |
| Docker Engine      | [Docker Installation Guide](https://docs.docker.com/engine/install/)                                                                        |
| NVIDIA Toolkit     | [NVIDIA Container Toolkit Installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)               |

## Required Docker Images

Ensure the following Docker images are built and available on your system before running the simulation:

| Image               | Description                                                                                                                                                        |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [space-robots](https://github.com/space-ros/docker/tree/main/space_robots) | The base Docker image for space robotics research.                                           |
| [moveit2](https://github.com/space-ros/docker/tree/main/moveit2)         | Provides MoveIt2 for motion planning.                                                            |
| [space ros](https://github.com/space-ros/space-ros)                      | Core ROS2 components for space robotics.                                                                        |

## Building and Running the Container

Start the Docker with
```
sudo systemctl start docker
```

To build and launch the container you should cd to `docker` folder:

To build image use this script:
```bash
./build.sh
```

To run docker container use this script:
```bash
./run.sh
```
That's all you need to start the simulation environment.

### Cloning Repositories into Docker

The following repositories are cloned into the Docker container during the build process:

1. **[isaac_ros2_utils](https://github.com/AlexanderRex/isaac_ros2_utils)**
2. **[curiosity_rover_description](https://github.com/AlexanderRex/curiosity_rover_description)**
3. **[mars_rover_control](https://github.com/AlexanderRex/mars_rover_control)**

## Recommended Development Setup

For an optimized development workflow, it is recommended to use **Visual Studio Code (VSCode)** along with its Docker-related extensions. These tools greatly simplify the process of interacting with Docker containers, especially when debugging and managing development environments.

### Recommended VSCode Extensions:
- **[Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers)**: Enables you to develop inside a Docker container.
- **[Docker Extension for VSCode](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)**: Simplifies the management of Docker containers directly from VSCode.

By using these tools, you can streamline your workflow and improve the experience of working within a Dockerized development environment.

## Usage

IMPORTANT: Before launching scripts, execute this command to attach to **space_ros_isaac_sim** container or use **VSCode remote connection**.
```bash
sudo docker exec -it space_ros_isaac_sim /bin/bash
```

In the first terminal, enter the Docker container(if not already) and launch Isaac Sim with one of the following two environments

- For spawning Curiosity in the sulfur stone area in the **Gediz Vallis channel** environment
   ```
   python ~/curiosity_sim/scripts/gale_crater_scene.py 
   ```
- For spawning Curiosity in the **test ground** environment
   ```
   python ~/curiosity_sim/scripts/test_scene.py
   ```
Note: DO NOT source demos_ws/install in this terminal (that launches Isaac Sim).


In the second terminal, enter the Docker container and launch the control system:
```bash
source ~/demos_ws/install/setup.bash
ros2 launch mars_rover_control mars_rover_control.launch.py
```

In the third terminal, for manual control using the keyboard:
```bash
source ~/demos_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Note: this terminal needs to be focused during keyboard control

### ROS Service Commands

In addition to ROS2 topics, the control graph supports various ROS2 service commands, allowing for precise control of different parts of the rover:

- **Drive the rover forward**:
    ```bash
    ros2 service call /move_forward std_srvs/srv/Empty
    ```

- **Stop the rover**:
    ```bash
    ros2 service call /move_stop std_srvs/srv/Empty
    ```

- **Turn left**:
    ```bash
    ros2 service call /turn_left std_srvs/srv/Empty
    ```

- **Turn right**:
    ```bash
    ros2 service call /turn_right std_srvs/srv/Empty
    ```

- **Open the tool arm**:
    ```bash
    ros2 service call /open_arm std_srvs/srv/Empty
    ```

- **Close the tool arm**:
    ```bash
    ros2 service call /close_arm std_srvs/srv/Empty
    ```

- **Open the mast (camera arm)**:
    ```bash
    ros2 service call /mast_open std_srvs/srv/Empty
    ```

- **Close the mast (camera arm)**:
    ```bash
    ros2 service call /mast_close std_srvs/srv/Empty
    ```

### Manuevering Curiosity on a Slope
There are many hills in the Gediz Vallis channel environment. Curiosity may drift downhill on a slope and it will require increased speed to counteract the incline.

### Recommended Render Settings for Isaac Sim

To optimize performance and maintain high FPS in the simulation, it is recommended to adjust the render settings in **Isaac Sim** as follows:

- **Enable DLSS**: Deep Learning Super Sampling (DLSS) is a technology that uses AI to upscale images, which can significantly boost performance without compromising visual quality. Enabling DLSS is especially useful for maintaining high FPS in complex scenes.
  - For more information, refer to the [NVIDIA DLSS documentation](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/rtx-renderer_rt.html#dlss).

- **Adjust Viewport Resolution**: If your system struggles to maintain a stable FPS, reducing the resolution of the **Viewport** window can help. Lowering the viewport resolution reduces the computational load, improving performance during simulation.
  - You can find more details on adjusting the viewport resolution in the [Viewport Settings documentation](https://docs.omniverse.nvidia.com/extensions/latest/ext_core/ext_viewport/controls/settings.html).

By using these settings, you can balance visual quality with performance, ensuring smoother operation of the simulation even on lower-end systems.



## Detailed Documentation

The following table provides links to detailed documentation on various components of the **Curiosity Mars Rover** simulation, covering control, sensors, and more.

| Document                  | Description                                                                                  |
|---------------------------|----------------------------------------------------------------------------------------------|
| [Maps](./docs/maps.md)                             | Information on maps and environments used in the simulation, providing context for different scenarios.|
| [Curiosity Description](./docs/curiosity_description.md) | Description and overview of the Curiosity rover model used in the simulation.|
| [Curiosity Control](./docs/curiosity_control.md)   | Detailed documentation on the control system of the Curiosity rover using ROS2 and action graphs.     |
| [Curiosity Sensors](./docs/curiosity_sensors.md)   | Explanation of the sensors used in the simulation, including Lidar, camera, and odometry configurations. |