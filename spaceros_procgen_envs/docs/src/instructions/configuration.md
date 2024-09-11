# Configuration

## Environment Configuration

The environments in Space ROS can be configured in two ways:

1. **Modifying the `configs/env.yaml` file**.
1. **Using environment variables**.

The default configuration file (`configs/env.yaml`) contains various settings that control the seed, scenario, level of detail, and options for assets (robot, object, terrain, vehicle).

### Example `env.yaml`:

```yaml
seed: 42 # SPACEROS_DEMO_SEED [int]
scenario: moon # SPACEROS_DEMO_SCENARIO [mars, moon, orbit]
detail: 0.125 # SPACEROS_DEMO_DETAIL [float]
assets:
  robot:
    variant: dataset # SPACEROS_DEMO_ASSETS_ROBOT_VARIANT [dataset]
  object:
    variant: procedural # SPACEROS_DEMO_ASSETS_OBJECT_VARIANT [primitive, dataset, procedural]
  terrain:
    variant: procedural # SPACEROS_DEMO_ASSETS_TERRAIN_VARIANT [none, primitive, dataset, procedural]
  vehicle:
    variant: dataset # SPACEROS_DEMO_ASSETS_VEHICLE_VARIANT [none, dataset]
```

### Setting Configuration via Environment Variables

To override specific values from the configuration file, environment variables can be passed during execution using the `run.sh` script. For instance:

```bash
spaceros_procgen_envs/run.sh -e SPACEROS_DEMO_DETAIL=1.0 -e SPACEROS_DEMO_SCENARIO=mars ros2 run spaceros_procgen_envs ...
```

This command sets the environment detail level to `1.0` and the scenario to `mars`.

## CLI Arguments

The following arguments are common across all entrypoint scripts (`teleop.py`, `agent_random.py`, `agent_zero.py`, `run_spaceros.py`):

- `-h`, `--help`: Display the help message and exit.
- `--task TASK`, `--demo TASK`, `--env TASK`: Specify the name of the task or environment. You can list available tasks using `list_envs.py`.
- `--num_envs NUM_ENVS`: Number of parallel environments to simulate.
- `--disable_ui`: Disable the majority of the Isaac Sim UI.
- `--headless`: Force the display to remain off, making the simulation headless.
- `--device DEVICE`: Set the device for simulation (e.g., `"cpu"`, `"cuda"`, or `"cuda:N"` where `N` is the device ID).

## Additional Environment Variables

### `SPACEROS_DEMO_SKIP_REGISTRATION`

By setting this to `"true"` or `1`, you can skip registering environments with the Gymnasium registry. This can be useful in specific deployment or testing scenarios.
