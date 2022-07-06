# Space ROS Demos

This repository provides examples for running robots using Space ROS


## Building the Demo Docker

The demos run in the Space ROS Demo docker. Please proceed to this [repo](https://github.com/tonylitianyu/docker-images/tree/tonylitianyu/demo_depends), clone and perform the following steps to build the docker image.

Within folder ```/spaceros```, run:

```
$ ./build-image.sh
```

Within folder ```/demo_spaceros```, run:

```
$ ./build-image.sh
```

## Running the Demo Docker

Come back to this repo, run the following to allow GUI passthrough:
```
$ xhost +local:docker
```

Then run:
```
$ ./run_sp.sh
```

Depends on the host computer, you might need to remove ```--gpus all``` flag in ```run_sp.sh```, which uses your GPUs

## Running the Demo

Make sure packages are sourced

```
$ source /root/src/spaceros_ws/install/setup.bash
```

```
$ source /root/src/depends_ws/install/setup.bash
```

Enter the folder ```/spaceros_demo_ws```, run:
```
$ rm -rf build install log && colcon build && . install/setup.bash
```

Launch the demo:
```
$ ros2 launch mars_rover mars_rover.launch.py
```

On the top left corner, click on the refresh button to show camera feed

## Perform Tasks

### Curiosity Rover

Open a new terminal and attach to the currently running container

```
$ docker -it exec <container-name> bash
```

Make sure packages are sourced

```
$ source /root/src/spaceros_ws/install/setup.bash
```

```
$ source /root/src/depends_ws/install/setup.bash
```

Explore randomly on the Mars surface:

```
$ ros2 service call /random_walk std_srvs/srv/Empty 
```

Open the tool arm:

```
$ ros2 service call /open_arm std_srvs/srv/Empty 
```

Close the tool arm:

```
$ ros2 service call /close_arm std_srvs/srv/Empty 
```



