import datetime
import importlib
import inspect
import os
from typing import Any, Dict, Union

import gymnasium as gym
import yaml
from omni.isaac.lab.utils import update_class_from_dict, update_dict

from spaceros_procgen_envs.core.envs import BaseEnvCfg


def load_cfg_from_registry(
    task_name: str, entry_point_key: str, unpack_callable: bool = True
) -> Union[
    BaseEnvCfg,
    Dict[str, Any],
]:
    """Load default configuration given its entry point from the gym registry.

    This function loads the configuration object from the gym registry for the given task name.
    It supports both YAML and Python configuration files.

    It expects the configuration to be registered in the gym registry as:

    .. code-block:: python

        gym.register(
            id="My-Awesome-Task-v0",
            ...
            kwargs={"env_entry_point_cfg": "path.to.config:ConfigClass"},
        )

    The parsed configuration object for above example can be obtained as:

    .. code-block:: python

        from spaceros_procgen_envs.utils.parsing import load_cfg_from_registry,

        cfg = load_cfg_from_registry("My-Awesome-Task-v0", "env_entry_point_cfg")

    Args:
        task_name: The name of the environment.
        entry_point_key: The entry point key to resolve the configuration file.

    Returns:
        The parsed configuration object. This is either a dictionary or a class object.

    Raises:
        ValueError: If the entry point key is not available in the gym registry for the task.
    """
    # Obtain the configuration entry point
    cfg_entry_point = gym.spec(task_name).kwargs.get(entry_point_key)
    # Check if entry point exists
    if cfg_entry_point is None:
        raise ValueError(
            f"Could not find configuration for the environment: '{task_name}'."
            f" Please check that the gym registry has the entry point: '{entry_point_key}'."
        )
    # Parse the default config file
    if isinstance(cfg_entry_point, str) and cfg_entry_point.endswith(".yaml"):
        if os.path.exists(cfg_entry_point):
            # Absolute path for the config file
            config_file = cfg_entry_point
        else:
            # Resolve path to the module location
            mod_name, file_name = cfg_entry_point.split(":")
            mod_path = os.path.dirname(importlib.import_module(mod_name).__file__)
            # Obtain the configuration file path
            config_file = os.path.join(mod_path, file_name)
        # Load the configuration
        print(f"[INFO]: Parsing configuration from: {config_file}")
        with open(config_file, encoding="utf-8") as f:
            cfg = yaml.full_load(f)
    else:
        if unpack_callable and callable(cfg_entry_point):
            # Resolve path to the module location
            mod_path = inspect.getfile(cfg_entry_point)
            # Load the configuration
            cfg_cls = cfg_entry_point()
        elif isinstance(cfg_entry_point, str):
            # Resolve path to the module location
            mod_name, attr_name = cfg_entry_point.split(":")
            mod = importlib.import_module(mod_name)
            cfg_cls = getattr(mod, attr_name)
        else:
            cfg_cls = cfg_entry_point
        # Load the configuration
        print(f"[INFO]: Parsing configuration from: {cfg_entry_point}")
        cfg = cfg_cls() if unpack_callable and callable(cfg_cls) else cfg_cls
    return cfg


def parse_task_cfg(
    task_name: str,
    device: str = "cuda:0",
    num_envs: int | None = None,
    use_fabric: bool | None = None,
) -> Union[
    BaseEnvCfg,
    Dict[str, Any],
]:
    """Parse configuration for an environment and override based on inputs.

    Args:
        task_name: The name of the environment.
        device: The device to run the simulation on. Defaults to "cuda:0".
        num_envs: Number of environments to create. Defaults to None, in which case it is left unchanged.
        use_fabric: Whether to enable/disable fabric interface. If false, all read/write operations go through USD.
            This slows down the simulation but allows seeing the changes in the USD through the USD stage.
            Defaults to None, in which case it is left unchanged.

    Returns:
        The parsed configuration object. This is either a dictionary or a class object.

    Raises:
        ValueError: If the task name is not provided, i.e. None.
    """
    # Create a dictionary to update from
    args_cfg = {"sim": {}, "scene": {}}

    # Simulation device
    args_cfg["sim"]["device"] = device

    # Disable fabric to read/write through USD
    if use_fabric is not None:
        args_cfg["sim"]["use_fabric"] = use_fabric

    # Number of environments
    if num_envs is not None:
        args_cfg["scene"]["num_envs"] = num_envs

    # Load the default configuration
    cfg = load_cfg_from_registry(task_name, "task_cfg", unpack_callable=False)
    # Update the main configuration
    if callable(cfg):
        default_cfg = cfg()
        cfg = cfg(
            sim=default_cfg.sim.replace(**args_cfg["sim"]),
            scene=default_cfg.scene.replace(**args_cfg["scene"]),
        )
    elif isinstance(cfg, dict):
        cfg = update_dict(cfg, args_cfg)
    else:
        update_class_from_dict(cfg, args_cfg)

    return cfg


def create_logdir_path(
    algo_name: str,
    task_name: str,
    prefix: str = "logs/",
    timestamp_format="%Y%m%d-%H%M%S",
) -> str:
    timestamp = datetime.datetime.now().strftime(timestamp_format)
    logdir = os.path.realpath(os.path.join(prefix, algo_name, task_name, timestamp))
    os.makedirs(logdir, exist_ok=True)
    return logdir
