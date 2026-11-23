

# Helicopter Flight Control Plugin

## Purpose
The `helicopter_flight_control` plugin is responsible for controlling the altitude of the Ingenuity helicopter in the Gazebo simulation. It implements a PID (Proportional-Integral-Derivative) controller to maintain a desired altitude by adjusting the collective pitch of the rotor blades.

## Key Components

1. **PID Controller**: 
   - Manages altitude control
   - Parameters: `Kp_alt`, `Ki_alt`, `Kd_alt` (proportional, integral, and derivative gains)

2. **Altitude Management**:
   - `desired_altitude`: Target altitude for the helicopter
   - `current_altitude`: Current altitude of the helicopter
   - `current_velocity_alt`: Current vertical velocity

3. **Control Output**:
   - Publishes the collective angle of attack for the rotor blades

## Inputs

1. **Configuration Parameters** (set in the SDF file):
   - `kp_alt`, `ki_alt`, `kd_alt`: PID controller gains
   - `desired_altitude`: Initial desired altitude
   - `link_name`: Name of the link to which forces and torques are applied

2. **Runtime Inputs**:
   - `/desired_altitude` topic: Allows dynamic changes to the target altitude
 
## Outputs

1. `/angle_of_attack` topic: 
   - Publishes the computed angle of attack for the rotor blades
   - Range: 2 to 8 degrees (0.0349066 to 0.139626 radians)

## Usage

```bash
ign topic -t /desired_altitude -m ignition.msgs.Double -p "data: 25.0"
```

```xml

<model name="ingenuity_model">
  <!-- Other model elements... -->

  <plugin name="gz::sim::systems::HelicopterControl" filename="helicopter_flight_control">
    <kp_alt>0.027</kp_alt>
    <ki_alt>0.0005</ki_alt>
    <kd_alt>0.1</kd_alt>
    <desired_altitude>10.0</desired_altitude>
    <link_name>body</link_name>
  </plugin>

  <!-- Other plugins and model elements... -->
</model>

```