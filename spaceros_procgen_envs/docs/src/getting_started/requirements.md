# System Requirements

<div class="warning">
This demo requires a dedicated NVIDIA GPU with RT Cores (RTX series). Isaac Sim does not support GPUs from other vendors.
</div>

## Hardware Requirements

The hardware requirements for running this simulation are inherited from the [Isaac Sim requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html). While it is possible to run the simulation on lower-spec systems than those recommended, performance will be significantly reduced.

| Component    | Minimum Requirement                   |
| ------------ | ------------------------------------- |
| Architecture | `x86_64`                              |
| CPU          | Anything is "good enough"             |
| RAM          | 16 GB                                 |
| GPU          | NVIDIA GPU with RT Cores (RTX series) |
| VRAM         | 4 GB                                  |
| Disk Space   | 30 GB                                 |
| Network      | 12 GB (for pulling Docker images)     |

## Software Requirements (Docker)

The following software requirements are essential for running the simulation inside a Docker container. Other operating systems or window managers may work, but significant adjustments may be required.

| Component      | Requirement                                         |
| -------------- | --------------------------------------------------- |
| OS             | Linux-based distribution (e.g., Ubuntu 22.04/24.04) |
| Window Manager | X11                                                 |
| NVIDIA Driver  | 535.183.01 (tested; other versions may work)        |
