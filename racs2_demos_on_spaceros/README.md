# RACS2 on Space ROS and Space Robots Demo Docker Image

The RACS2 on Space ROS and Space Robots Demo docker image uses the space_robots demo docker image (*osrf/space-ros:curiosity_demo*) as its base image.
Build instructions for that image can be found in [this README](../curiosity_rover/README.md).
The Dockerfile installs all of the prerequisite system dependencies along with the demos source code, then builds the Space ROS Space Robots and RACS2 demo.

This is RACS2 Bridge demo for Curiosity Mars rover.

## Building the Demo Docker

This demo builds on top of the [`space-ros`](https://github.com/space-ros/space-ros) and [`curiosity_rover`](https://github.com/space-ros/demos/tree/main/curiosity_rover) Docker images.

To build the RACS2 demo image, follow these steps:

```bash
git clone https://github.com/space-ros/demos.git
cd demos/curiosity_rover
./build.sh
cd ../racs2_demos_on_spaceros
./build.sh
```

The final step builds the racs2_demos_on_spaceros image using the base images built in the previous steps.

Make sure you have Docker installed and sufficient permissions to run it (sudo may be required depending on your setup).

## Running the Demo Docker

At /path/to/demos/racs2_demos_on_spaceros, run:

```bash
./run.sh
```

The above command launch racs2 bridge node.

### Running cFS bridge app & run_app app
Open a new terminal (calling Terminal 1) and attach to the currently running service:

```bash
docker compose exec -it racs2_demo bash
cd ~/racs2_ws
cd cfs/build/exe/cpu1/
./core-cpu1
```

**Executing commands to the rover must be done with this terminal active.**

### Available Commands

Drive commands to the rover are input via keyboard in Terminal 1. The keymap is as follows.

* "w": Drive the rover forward
* "s": Drive the rover backward
* "a": Turn left
* "d": Turn right
* "x": Stop the rover

### Nodes

![RACS2 demo on Space ROS Mars rover demo](racs2_demo_on_spaceros_nodes.png)

## Reference

* [RACS2 bridge project by Japan Aerospace Exploration Agency (JAXA)](https://github.com/jaxa/racs2_bridge)

* [Hiroki Kato and Tatsuhiko Saito, "RACS2: the ROS2 and cFS System - launched" Flight Software Workshop 2023.](https://drive.google.com/drive/folders/1C9fokWGDl2e4NfgX_ZU3f98FfPe9udwQ)

* [Hiroki Kato and Tatsuhiko Saito, "ROS and cFS System (RACS): Easing Space Robotic Development post-opensource activities and ROS2 integration" Flight Software Workshop 2021.](https://drive.google.com/file/d/11L48doT_pRNs7R0hdChPALqJO849TvV2/view?usp=drive_web)
