

## Ingenuity Description

This folder contains the model description files for the Ingenuity helicopter.
- `ingenuity_description.urdf`: URDF file describing the Ingenuity model used for RViz visualization and tf publisher
- `ingenuity_world.sdf`: SDF file describing the simulation world
- Mesh files for visual representation
- Blender mesh of mars_tharsis_region
    - Added texture and some rocks and optimized the size

The following system plugins from Gazebo are used in the simulation:

1. Martian Dust Storm:
   To start the Martian dust storm:
   ```bash
   ign topic -t /model/dust_storm/link/dust_link/particle_emitter/emitter/cmd -m ignition.msgs.ParticleEmitter -p 'emitting: {data: true}'
   ```

2. Get the Battery State:

    ```bash
    ign topic -et /model/ingenuity_model/battery/ingenuity_battery/state
    ```

3. Command rotor angular velocity

    ```bash
    ign topic -t "/model/ingenuity_model/joint/rod_1_joint/cmd_vel" -m ignition.msgs.Double -p "data: 2000.0"
    ign topic -t "/model/ingenuity_model/joint/rod_2_joint/cmd_vel" -m ignition.msgs.Double -p "data: 2000.0"
    ```

4. Wind

    Must be enabled in the sdf file under ingenuity_description. Allows user to send linear wind forces.
    
    ./ingenuity_flight_simulator/ingenuity_description/sdf/ingenuity_world.sdf


## Sensors
# Ingenuity Sensors and Data Access

The Ingenuity simulation includes several sensors to mimic the real Mars helicopter. Here's a detailed explanation of each sensor and how to access its data using the `ign topic -et` command.

## 1. RGB Camera

The RGB camera simulates Ingenuity's downward-facing camera, used for navigation and terrain imaging.


### Accessing Camera Data:
```bash
ign topic -et /camera
```

This command will stream the camera images. Each message includes the image data, timestamp, and other metadata.

## 2. Altimeter

The altimeter sensor provides altitude measurements, crucial for Ingenuity's vertical positioning.

### Specifications:
- Update Rate: 30 Hz
- Noise: Gaussian noise applied to both position and velocity measurements

### Accessing Altimeter Data:
```bash
ign topic -et /altimeter
```

This command will stream altimeter data, including vertical position and velocity.

## 3. IMU (Inertial Measurement Unit)

The IMU provides data about the helicopter's orientation and acceleration.

### Specifications:
- Update Rate: 100 Hz
- Measurements: Linear acceleration and angular velocity in 3 axes

### Accessing IMU Data:
```bash
ign topic -et /imu
```

This command will stream IMU data, including linear acceleration and angular velocity vectors.

## Additional Sensor Data

### Battery State:
To monitor the battery state of the Ingenuity model:
```bash
ign topic -et /model/ingenuity_model/battery/ingenuity_battery/state
```

This provides information about the current charge level, voltage, and other battery parameters.

### Joint States:
To view the joint states of the Ingenuity model:
```bash
ign topic -et /world/ingenuity_world/model/ingenuity_model/joint_state
```

This gives information about the position and velocity of the model's joints, including the rotor joints.

### Model Pose:
To get the current pose of the Ingenuity model:
```bash
ign topic -et /model/ingenuity_model/pose
```

This provides the position and orientation of the Ingenuity model in the world frame.
