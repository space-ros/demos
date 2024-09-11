# SSRMS CanadArm2 - Demo

This is a simple demo of controlling the canadarm using spaceROS.

### Installation

To start the demo, there are few dependencies that need to be installed. The following steps will guide you through the installation process.

1. You will need docker installed on your system. If not, you can follow the instructions [here](https://docs.docker.com/get-docker/).

### How to run the demo

1. Clone the demo repository
    ```bash
    git clone https://github.com/space-ros/demos.git
    ```
2. cd into the `canadarm2` directory
    ```bash
    cd canadarm2
    ```
3. To build the demo, you can use the following command.
    ```bash
    # To build the demos for the canadarm2
    ./build.sh
    ```
4. To run the demo, you can use the following command.
    ```bash
    # To run the demos for the canadarm2
    ./run.sh
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
