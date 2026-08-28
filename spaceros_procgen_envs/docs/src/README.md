# Introduction

The `spaceros_procgen_envs` package offers a collection of procedurally generated environments designed to bridge the gap between simulated robotics and the vast unpredictability of space.

![](../_images/title.png)

## Key Features

### On-Demand Procedural Generation with [Blender](https://blender.org)

This package utilizes the open-source Blender to procedurally generate unique 3D assets for each environment. By doing so, it enables Space ROS developers to simulate a wide range of scenarios, thereby enhancing the robustness and generalization capabilities of their algorithms, without the need to hoard terabytes of static datasets.

### Highly-Parallelized Simulation with [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)

All environments support highly parallel simulation instances, significantly accelerating workflows such as parameter tuning, verification, synthetic data generation, and online learning. The uniqueness of each procedurally generated instance ensures a diverse range of experiences, making it possible to capture edge cases through domain randomization. Furthermore, building on top of [Isaac Lab](https://isaac-sim.github.io/IsaacLab) enhances compatibility with a wide array of pre-configured robots and sensors.

### Compatibility with [Gymnasium API](https://gymnasium.farama.org)

The package employs a standardized interface through the Gymnasium API, ensuring seamless integration with a broad ecosystem of libraries and tools. This enables developers to leverage popular reinforcement learning and imitation learning algorithms while also simplifying the evaluation and comparison of various solutions across diverse scenarios, giving rise to potential collaborations and benchmarking efforts.

### Integration with [Space ROS](https://space.ros.org)

The package is interoperable with Space ROS and the broader ROS ecosystem, providing access to a rich set of tools and libraries that accelerate the development and deployment of robotic systems. Its compatibility with ROS tooling ensures easy integration into existing workflows. Additionally, it supports multiple parallel instances via namespaced middleware communication, enabling multi-robot operations.

### Agnostic Interfaces

The package was designed with abstraction layers to ensure flexibility for various space robotics applications. By adjusting or replacing procedural pipelines, a single task definition can be adapted for use across different robots and planetary or orbital environments. Moreover, the procedural asset generation pipeline is decoupled from the core package, allowing for integration with external frameworks or standalone use.

### Diverse Demonstrations

The initial release of the package includes multiple robot-enabled demonstrations in diverse space environments, highlighting the potential of procedural generation and parallel simulation. Additionally, the package features a set of goal-oriented tasks, demonstrating its versatility for developing autonomous decision-making solutions.
