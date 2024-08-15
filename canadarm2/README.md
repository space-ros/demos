# Curiosity Rover - Demo

This is a simple demo of controlling the canadarm using spaceROS.

### Installation

To start the demo, there are few dependencies that need to be installed. The following steps will guide you through the installation process.

1. You will need ROS Humble on your host system. If it is not already installed, you can follow the instructions [here](https://docs.ros.org/en/humble/Installation.html).
2. You will need Gazebo Sim installed on your system. If not, you can follow the instructions [here](https://gazebosim.org/docs/all/getstarted/).
3. You will need docker installed on your system. If not, you can follow the instructions [here](https://docs.docker.com/get-docker/).
4. Install xterm for running the demo. You can install it using the following command.
    ```bash
    sudo apt-get install xterm
    ```

### How to run the demo

1. Clone the demo repository
    ```bash
    git clone http://github.com/space-ros/demo.git
    ```
2. cd into the `canadarm2` directory
    ```bash
    cd canadarm2
    ```
3. To build and run the demo, we use `Makefile`. Run the following command to build the docker image and run the container.
    ```bash
    # To see the list of available commands
    make help

    # To build and run the demo
    make run
    ```

This will start the demo in one terminal and gazebo in another terminal. To control the canadarm2, we provide ros2 services for the demo. You can control the rover using the following services.

1. Control arm of the canadarm

    ```bash
    # To open an arm in outstretched position
    ros2 service call /open_arm std_srvs/srv/Empty

    # To move the arm to its default close pose of all zeros
    ros2 service call /close_arm std_srvs/srv/Empty

    # To move the arm to a random pose
    ros2 service call /random_arm std_srvs/srv/Empty
    ```
