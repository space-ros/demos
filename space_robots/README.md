# Space Robots Demo Docker Image

This folder contains the Dockerfile and scripts required to build the Curiosity Mars rover and Canadarm demos into an image called `space_robots_demo`.

The Dockerfile installs all of the prerequisite system dependencies along with the demos source code, then builds the demo code.

The image uses [osrf/space-ros-moveit2:latest](https://hub.docker.com/r/osrf/space-ros-moveit2/tags) as its base image.

**Note:** This demo was moved from the [space-ros/docker](https:/github.com/space-ros/docker) repository. For history prior to the move, search that repo.

## Building the Demo Image

Use the build script provided to build the docker image.

```
./build.sh
```

## Running the image container

Run the following to allow GUI passthrough:

```bash
xhost +local:docker
```

Then run:

```bash
./run.sh
```

## Running the Demos

### Curiosity Mars rover demo

1. Launch the demo in Gazebo:

```bash
ros2 launch curiosity_gazebo curiosity_gazebo.launch.py
```

On the top left corner, click on the refresh button to show camera feed.

2. Launch the ROS 2 control nodes
Open a new terminal and attach to the currently running container:

```bash
docker exec -it openrobotics_space_robots_demo bash
```
Within the container, launch the control nodes:
```
ros2 launch curiosity_rover_demo mars_rover.launch.py
```

3. Send commands to the rover
Open a new terminal and attach to the currently running container:

```bash
docker exec -it openrobotics_space_robots_demo bash
```
Within the container, you can now move the rover using the commands in [demos/curiosity_rover/README.md](../curiosity_rover/README.md)

#### Canadarm demo
Run the demo container as shown above:
```bash
./run.sh
```

1. Launch the canadarm demo in Gazebo
```bash
ros2 launch canadarm_gazebo canadarm.launch.py
```
2. Launch the ROS 2 control node
Open a new terminal and attach to the currently running container:

```bash
docker exec -it openrobotics_space_robots_demo bash
```
Within the container, launch the control node:

```bash
ros2 launch canadarm_demo canadarm.launch.py
```
Within the container, you can now move the arm using the commands in [demos/canadarm2/README.md](../canadarm2/README.md)
