# NASA Space ROS Sim Summer Sprint Challenge

Team Lead Freelancer Username: SanjayJP02

Submission Title: CADRE Demo: Cooperative Autonomous Distributed Robotic Exploration

# DPackage Description and Purpose

- This package simulates NASA’s CADRE mission, which focuses on mapping the Moon through a trio of autonomous rovers. The simulation is developed using ROS 2 and is part of an open-sourced lunar exploration initiative, slated for Spring 2026.


The package consists of the following:

- Lunar Landscape Design: Created a realistic lunar environment in Blender, based on publicly available NASA Moon images.
- CAD Model Development: Designed the CADRE rover model in SolidWorks and converted it into URDF format for ROS 2 Humble compatibility.
- Simulation & Integration: Integrated the rover with IMU, LRF, and RGB-D sensors for lunar terrain mapping and exploration in Gazebo.
- Teleoperation & Control: Built a Python-based GUI for rover teleoperation, featuring camera visualization and real-time control.
- Cross-Platform Compatibility: Ensured simulation compatibility across WSL and VMware Linux for flexible development.

Future Work:

- Implementing reinforcement learning for coordinated mapping and exploration in Gazebo.

# Definition and description of the public API


The CADRE rover topics:
-/clicked_point
-/clock
-/goal_pose
-/initialpose
-/parameter_events
-/performance_metrics
-/robot_1/cmd_vel
-/robot_1/joint_states
-/robot_1/odom
-/robot_1/robot_1_camera/camera_info
-/robot_1/robot_1_camera/depth/camera_info
-/robot_1/robot_1_camera/depth/image_raw
-/robot_1/robot_1_camera/depth/image_raw/compressed
-/robot_1/robot_1_camera/depth/image_raw/compressedDepth
-/robot_1/robot_1_camera/depth/image_raw/theora
-/robot_1/robot_1_camera/image_raw
-/robot_1/robot_1_camera/image_raw/compressed
-/robot_1/robot_1_camera/image_raw/compressedDepth
-/robot_1/robot_1_camera/image_raw/theora
-/robot_1/robot_1_camera/points
-/robot_1_front/scan
-/robot_1_imu/data
-/robot_1_robot_description
-/robot_2/cmd_vel
-/robot_2/joint_states
-/robot_2/odom
-/robot_2/robot_2_camera/camera_info
-/robot_2/robot_2_camera/depth/camera_info
-/robot_2/robot_2_camera/depth/image_raw
-/robot_2/robot_2_camera/depth/image_raw/compressed
-/robot_2/robot_2_camera/depth/image_raw/compressedDepth
-/robot_2/robot_2_camera/depth/image_raw/theora
-/robot_2/robot_2_camera/image_raw
-/robot_2/robot_2_camera/image_raw/compressed
-/robot_2/robot_2_camera/image_raw/compressedDepth
-/robot_2/robot_2_camera/image_raw/theora
-/robot_2/robot_2_camera/points
-/robot_2_front/scan
-/robot_2_imu/data
-/robot_2_robot_description
-/robot_3/cmd_vel
-/robot_3/joint_states
-/robot_3/odom
-/robot_3/robot_3_camera/camera_info
-/robot_3/robot_3_camera/depth/camera_info
-/robot_3/robot_3_camera/depth/image_raw
-/robot_3/robot_3_camera/depth/image_raw/compressed
-/robot_3/robot_3_camera/depth/image_raw/compressedDepth
-/robot_3/robot_3_camera/depth/image_raw/theora
-/robot_3/robot_3_camera/image_raw
-/robot_3/robot_3_camera/image_raw/compressed
-/robot_3/robot_3_camera/image_raw/compressedDepth
-/robot_3/robot_3_camera/image_raw/theora
-/robot_3/robot_3_camera/points
-/robot_3_front/scan
-/robot_3_imu/data
-/robot_3_robot_description
-/rosout
-/tf
-/tf_static


More Rovers can be added by using the following modifttcionin main.launch.xml:

```xml
<include file="$(find-pkg-share robot_gazebo)/launch/spawn_with_control.launch.xml">
    <arg name="robot_name" value="robot_#"/>
    <arg name="robot_file" value="robot.xacro"/>
    <arg name="x_spawn" value="0.0"/>
    <arg name="y_spawn" value="0.0"/>
    <arg name="z_spawn" value="0.0"/>
    <arg name="roll_spawn" value="0.0"/>
    <arg name="pitch_spawn" value="0.0"/>
    <arg name="yaw_spawn" value="0.0"/>
</include>
```

# How to build and install

To build the docker image and start a docker container follow these passages

1. Clone this repository, change directory to demos\CADRE_demo

2. Run the build script ```./build.sh```

3. Start the container by executing ```./run.sh```

4. (optional) additional terminals can be opened executing ```./open_cmd.sh```

# Run CADRE demo

The CADRE demo can be started by executing the following commands:

Run ```ros2 launch robot_gazebo main.launch.xml``` this launch file will start the gazebo simulator with the Three CADRE rovers in Lunar world.


# Run CADRE GUI teleopration.

In another terminal run ```ros2 run robot_control joy.py``` this will start GUI control for controlling individual Rovers.

# Run CADRE point clouds.

In another terminal run ```ros2 run robot_control mapping.py``` this will start point cloud node and publishes topics.



# License

Apache License 2.0