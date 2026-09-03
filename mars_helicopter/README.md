# NASA Space ROS Sim Summer Sprint Challenge

Team Lead Freelancer Username: @exp99generator

Submission Title: Ingenuity coaxial helicopter model

# Description and purpose of package

This package aims to accelerate the development of guidance, navigation, and control algorithms for autonomous flying vehicles utilizing a helicopter architecture.

![Ingenuity](/mars_helicopter/assets/ingenuity.png)

The package consists of the following:
- A Gazebo world modeled after the Jezero Mars crater, and a coaxial helicopter SDF model of Ingenuity (available in the space-ros/simulation repository).
- A Gazebo plugin that simulates rotor disk flapping dynamics, rotor inflow, and rotor forces. The rotor can be controlled by varying the collective pitch, longitudinal and lateral cyclic pitch, and angular velocity through messages sent via Gazebo topics.
- A Gazebo plugin for the atmosphere, which starts a service that allows the rotors in the simulation to poll relevant information, such as density at a certain height (currently modeled with uniform density).

The implementation uses mathematical models from:
- Principles of Helicopter Aerodynamics by J. Gordon Leishman
- Helicopter Flight Dynamics by Gareth D. Padfield

# Definition and description of the public API

The rotor plugin subscribes to the following gazebo topics:
- /{link_name}/angular_velocity (ignition.msgs.Double)
- /{link_name}/collective (ignition.msgs.Double)
- /{link_name}/lateral_cyclic (ignition.msgs.Double)
- /{link_name}/longitudinal_cyclic (ignition.msgs.Double)

The Ingenuity helicopter publishes sensor data on:
- /ingenuity/sensors/imu
- /ingenuity/sensors/laser_altimeter/range
- /ingenuity/sensors/laser_altimeter/range/points
- /ingenuity/sensors/mono_camera/camera_info
- /ingenuity/sensors/mono_camera/image_raw
- /ingenuity/sensors/rgb_camera/camera_info
- /ingenuity/sensors/rgb_camera/image_raw

The atmosphere plugin opens a service at:
- /atmo/density

The plugins can be configured by using the following SDF elements:
- rotor plugin: 
```xml
<plugin name="simulation::RotorPlugin" filename="libRotorPlugin.so">
    <!-- Plugin parameters (if any) -->
    <link_name>fuselage_link</link_name>
    <joint_name>gazebo_joint_name</joint_name>
    <link_to_hub_frame> 0 0 0 0 0 0</link_to_hub_frame> <!-- relative pose between link and hub frame -->
    <force_model>base</force_model> <!-- base -->
    <flapping_model>gordon</flapping_model> <!-- center_spring, gordon -->
    <inflow_model>harmonic_1</inflow_model> <!-- harmonic_1 -->
    <config>
    <blade>
        <chord>0.1029</chord> <!-- m -->
        <radius>0.514334</radius> <!-- m -->
        <lift_slope>5.73</lift_slope> <!-- 1/rad -->
        <drag_quadric>0.14875</drag_quadric>
        <drag_constant>0.0738</drag_constant> <!-- Cd = constant + quadric*Cl^2 -->
        <twist_slope>0.01</twist_slope> <!-- blade twist rad -->
        <blade_inertia>0.005</blade_inertia> <!-- kg*m^2 -->
    </blade>
    <direction>cw</direction> <!-- cw=clockwise, ccw=counter-clockwise -->
    <hinge_stiffness>4.5680</hinge_stiffness> <!-- Nm/rad -->
    <num_blades>2</num_blades>
    <rotor_inertia>10</rotor_inertia> <!-- kg*m^2 -->
    </config>
</plugin>
```

- atmosphere plugin:

```xml
<plugin name="simulation::AtmospherePlugin" filename="libAtmospherePlugin.so">
    <model>uniform_density</model> <!-- uniform_density -->
    <surface_density>0.020</surface_density> <!-- kg/m^3 -->
</plugin>
```

# How to build and install

To build the docker image and start a docker container follow these passages

1. Clone this repository, change directory to demos\mars_helicopter\docker

2. Run the build script ```./build.sh```

3. Start the container by executing ```./run.sh```

4. (optional) additional terminals can be opened executing ```./open_cmd.sh```

# Run Ingenuity demo

The Ingenuity helicopter demo can be started by executing the following commands:

1. Run ```ros2 launch mars_helicopter mars_helicopter.launch.py``` this launch file will start the gazebo simulator with the Jezero crater world and will initiate the gazebo/ros2 bridge

2. In another terminal run ```ros2 run mars_helicopter teleop_helicopter``` this will start a simple ros2 teleop node.

3. Command the helicopter:
    * Enter for rotor on/off
    * Space bar for collective
    * WASD for cyclic input
    * HL for yaw control by differential collective input


# License

Apache License 2.0