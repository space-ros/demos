from importlib.util import find_spec


def is_sim_app_started() -> bool:
    return find_spec("omni.isaac.version") is not None
