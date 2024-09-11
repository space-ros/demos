

# Helicopter Flight Simulation

Contains the flight simulation gazebo plugin for the Ingenuity simulator.
- Simulates the aerodynamics of the blades
- Calculates lift and drag forces based on angle of attack and cyclic inputs
- Applies forces and torques to the Ingenuity model

The flight simulator for the Ingenuity helicopter takes various inputs, both at runtime and during configuration. Let's break these down:

## Runtime Inputs (Subscribed Topics)

1. `/angle_of_attack`: 
   - This topic receives messages about the overall angle of attack for the rotor blades.
   - It's used to control the collective pitch of the rotors.

2. `/alpha_c`:
   - This topic receives messages for the longitudinal cyclic control.
   - It affects the left/right tilt of the rotor disk.

3. `/alpha_s`:
   - This topic receives messages for the lateral cyclic control.
   - It affects the forward/backward tilt of the rotor disk.

These runtime inputs allow for real-time control of the helicopter's flight characteristics.

## Configuration Parameters

These parameters are set when the simulator is initialized:

1. `cla`: Coefficient of Lift / alpha slope
2. `cda`: Coefficient of Drag / alpha slope
3. `cma`: Coefficient of Moment / alpha slope
4. `alphaStall`: Angle of attack at which the airfoil stalls
5. `claStall`: Cl-alpha rate after stall
6. `cdaStall`: Cd-alpha rate after stall
7. `cmaStall`: Cm-alpha rate after stall
8. `rho`: Air density
9. `radialSymmetry`: Boolean indicating if the shape is aerodynamically radially symmetric
10. `area`: Effective planform surface area
11. `alpha0`: Initial angle of attack
12. `cp`: Center of pressure (3D vector)

## Outputs

The simulator publishes wrench (force and torque) data to a topic structured as:
`/wrench/<model_name>/<link_name>`

### Collective Control

To control the collective pitch of the rotors, publish to the `/angle_of_attack` topic:
```bash
ign topic -t /angle_of_attack -m ignition.msgs.Double -p "data: 0.1"
```
By publishing to the /desired_altitude topic the collective will be automatically adjusted by the ingenuity flight controller plugin

```bash
ign topic -t /desired_altitude -m ignition.msgs.Double -p "data: 42"
```

#### Cyclic Control:

Cyclic control changes the feathering angle of the blades as they rotate, creating an uneven lift that tilts the rotor disk. This tilt is what allows the helicopter to move in different directions.

The feathering angle (θ) at any azimuth angle (Ψ) is given by the equation:

```
θ(Ψ) = θ₀ + θc * cos(Ψ) + θs * sin(Ψ)
```

For helicopter flight simulator plugin the θc and θs are initialized with 0.0.
θ₀ is initialized with 0.1.

Where:
- θ₀ is the collective pitch (base angle for all blades)
- θc is the longitudinal cyclic pitch
- θs is the lateral cyclic pitch

To control Ingenuity's cyclic pitch in the simulation, use the following topics:

- Lateral cyclic: `/alpha_c`
- Longitudinal cyclic: `/alpha_s`

Example:
```bash
ign topic -t /alpha_c -m ignition.msgs.Double -p "data: 0.00001"
ign topic -t /alpha_s -m ignition.msgs.Double -p "data: 0.00001"
```

These commands allow you to adjust the cyclic control of Ingenuity, enabling more complex maneuvers beyond simple altitude changes.


## Usage

```xml

<model name="ingenuity_model">
  <!-- Other model elements... -->

  <plugin
    filename="helicopter_flight_simulation"
    name="gz::sim::systems::HelicopterFlightSimulation">
    <a0>0.1</a0>
    <cla>0.1</cla>
    <cda>0.001</cda>
    <cma>0.0</cma>
    <cp>0.0 0.5 0</cp>
    <area>0.2</area>
    <air_density>0.024082</air_density>
    <forward>1 0 0</forward>
    <upward>0 0 1</upward>
    <link_name>blade_1</link_name>
  </plugin>

  <!-- Repeat similar plugin configuration for blade_2, blade_3, and blade_4 -->

  <!-- Other plugins and model elements... -->
</model>

```