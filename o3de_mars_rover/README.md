# o3de_mars_rover

This directory contains the Dockerfile instructions for creating the ```TODO``` image that showcases Curiosity rover navigating through a clutterd test environment inspired by JPL's Mars Yard testing ground.

This project was created as a submission to the NASA Space ROS Sim Summer Sprint Challenge 2024. A short review on O3DE, a small video showing the system in action and other related discussion can be found in this [PR](https://github.com/space-ros/demos/pull/64)

## Prerequisits

* ```osrf/space-ros```. I will eventually upgrade the Dockerfile to use the ```openrobotics/moveit2::latest``` from [space-ros/docker/movit2](https://github.com/space-ros/docker/tree/main/moveit2) image

## Build instruction

**WARNING** At least have 60 GB of free disk space and since it builds Moveit2 and Demo_ws before building the whole O3DE project, expect at least 3 - 4 hours for the build process to complete.

* Git clone the top level ```demos``` repository and ```change directory``` into it. If you are just interested in the image, you don't need to clone this repo. Then from a terminal

```bash
cd /o3de_mars_rover
./build.sh
```

* Install Nvidia Container toolkit and run configuration as discussed here: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html. If you don't have it open a terminal and do the following

* Make sure you have --runtime nvidia configured. If not this sample example will do it: ```sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi```

## Run the demo

* In the terminal where you would launch the O3DE, run the following 

```bash
xhost +local:docker
./run.sh
```

* Open another terminal and find the 
