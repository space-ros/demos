# Parallel Environments

Running environments in parallel can be essential for tasks like:

- Testing the scalability of the system.
- Collecting synthetic data.
- Training reinforcement learning agents efficiently by leveraging multiple environments.

![](../_images/parallel_envs.png)

## The `--num_envs` Argument

The `teleop.py` script accepts an optional `--num_envs` argument. By default, this is set to `1`, but you can specify more environments for parallel execution. For example, to run four environments, use the following command:

```bash
spaceros_procgen_envs/run.sh ros2 run spaceros_procgen_envs teleop.py --task sample_collection --num_envs 4
```

Each environment will generate its own procedural assets, providing unique experiences across different simulations. However, note that the time taken to generate these assets scales linearly with the number of environments. These assets will be cached for future runs unless the cache is cleared (explained later in this document).

After the environments are initialized, they can be controlled in sync using the same keyboard scheme displayed in the terminal.

## Using Random and Zero Agents

Instead of manually controlling each environment, there are options to automate the process using agents. This is useful for testing whether environments function as expected.

### Random Agents

The `agent_random.py` script allows environments to act based on random actions sampled from the action space. This is particularly useful for verifying if environments are running as intended without manual control:

```bash
spaceros_procgen_envs/run.sh ros2 run spaceros_procgen_envs agent_random.py --task sample_collection --num_envs 4
```

### Zero Agents

Alternatively, `agent_zero.py` executes environments where all actions are zero-valued, mimicking a steady-state system. This can be useful for analyzing the idle behavior of environments:

```bash
spaceros_procgen_envs/run.sh ros2 run spaceros_procgen_envs agent_zero.py --task sample_collection --num_envs 4
```

Both agent scripts (`agent_random.py` and `agent_zero.py`) share the same core arguments as `teleop.py`, with the primary difference being that they do not require teleoperation interfaces.
