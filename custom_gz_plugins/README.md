# NASA Space ROS Sim Summer Sprint Challenge

Team Lead Freelancer Username: @utkrishtjain

Submission Title: Custom Models & Gazebo Plugins for Realistic Simulations

## Description and Features

### Day Light Manager Plugin
The Sun is a crucial factor in driving most planetary processes, affecting nearly every environmental aspect either directly or indirectly. For realistic environmental simulations, accurately modeling the Sun's movement is vital, particularly in complex scenarios. The Daylight Manager plugin in Gazebo simulates the Sun's path based on the time of day and the observer's latitude, offering a dynamic and lifelike portrayal of sunlight. As the Sun's position changes, the shadows cast by objects and the background color also shift, as illustrated below.
> For detailed description of the plugin and it's usage, click [here](src/DayLightManager/README.md).

![Day Light Manager](assets/time_of_day.gif)


### Dust Manager Plugin
The Dust Manager plugin enables the simulation of realistic dust behavior in Martian or space environments by managing particle emitters. Dust plays a critical role in extraterrestrial settings, especially on Mars, where it can impact visibility, equipment functionality, and the overall success of missions. This plugin allows for precise simulation of dust clouds, supporting the development and testing of vehicles and systems intended for planetary exploration. By replicating how dust interacts with moving objects, it helps create more authentic and challenging scenarios for space missions. Additionally, the Dust Manager Plugin offers the ability to control wind conditions based on the parameters of dust storms incorporated into the simulation.
> For detailed description of the plugin and it's usage, click [here](src/DustManager/README.md).

![Dust Manager Gui](assets/dust_gui.gif)


### Real Surface Models
Custom models made from real elevation data (From NASA's Planetary Data System) of celestial bodies like Moon and Mars is also added.

- Model of Lunar Terrain made from elevation data LOLA on board the Lunar Reconnaissance Orbiter (LRO)

- Models of Martian Terrain made from elevation data from HiRISE Images on board the Mars Reconnaissance Orbiter (MRO)

<img src="assets/mars_world.png" alt="Mars" width="45%">
<img src="assets/moon_world.png" alt="Moon" width="45%">

## Build and Run

- Build the Custom Docker image with all the custom dependencies using the following command

```shell
./build.sh
```

- Once the Image has been build, use the following command to run the container

```shell
./run.sh
```
This will run the container where you can launch the simulation.
> If the container is running, you can use the same script/command in another terminal instance to attach a terminal into the docker.

- Once the docker container is up, you can use the demo launch file to launch the simualtion using the following command.

```shell
ros2 launch custom_gz_plugins demo_world.launch.py
```








