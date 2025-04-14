# Curiosity Rover - Demo

This is a simple demo of controlling the curiosity rover using spaceROS.

### Installation

To start the demo, there are few dependencies that need to be installed. The following steps will guide you through the installation process.

1. You will need Nvidia's Isaac Sim installed on your system. If not, you can follow the instructions [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html).
2. You will need docker installed on your system. If not, you can follow the instructions [here](https://docs.docker.com/get-docker/).

### How to run the demo

1. Clone the demo repository
    ```bash
    git clone https://github.com/space-ros/demos.git
    ```
2. cd into the `curiosity_rover` directory
    ```bash
    cd curiosity_rover
    ```
3. To build the demo, you can use the following command.
    ```bash
    # To build the demos
    ./build.sh
    ```
4. To run the demo, you can use the following command.
    ```bash
    # To run the demo with gazebo
    ./run.sh

    # To run the demo with isaac sim
    ./run.sh --isaacsim
    ```

This will start the demo in one terminal and gazebo in another terminal. To control the canadarm2, we provide ros2 services for the demo. You can control the rover using the following services.

> NOTE: If you are using Isaac Sim, make sure to start the simulation in Isaac Sim before running the demo. You can find more information on how to start the simulation in Isaac Sim [here](https://github.com/space-ros/simulation/).


1. To move around, you can use teleop_twist_keyboard. To run the teleop_twist_keyboard, you can run the following command,
    ```bash
    # For gazebo
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

    # For isaacsim
    ros2 run teleop_twist_keyboard teleop_twist_keyboard -r /cmd_vel:=/curiosity/cmd_vel
    ```

    You can also use the following services to control the rover,
    ```bash
    # Move forward
    ros2 service call /move_forward std_srvs/srv/Empty

    # Turn left
    ros2 service call /turn_left std_srvs/srv/Empty

    # Turn right
    ros2 service call /turn_right std_srvs/srv/Empty

    # Stop
    ros2 service call /stop std_srvs/srv/Empty
    ```
2. To control the arm of the rover, you can run the following services,
    ```bash
    # Open the arm
    ros2 service call /open_arm std_srvs/srv/Empty

    # Close the arm
    ros2 service call /close_arm std_srvs/srv/Empty
    ```
3. To control the mast arm of the rover, you can run the following services,
    ```bash
    # Open the mast arm
    ros2 service call /mast_open std_srvs/srv/Empty

    # Close the mast arm
    ros2 service call /mast_close std_srvs/srv/Empty

    # To rotate the mast arm
    ros2 service call /mast_rotate std_srvs/srv/Empty
    ```