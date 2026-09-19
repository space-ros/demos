from os import environ

from .utils.importer import import_modules_recursively
from .utils.sim_app import is_sim_app_started

## Verify the availability of the Rust extension module without writing its bytecode to disk
try:
    from . import _rs  # noqa: F401
except Exception:
    raise ModuleNotFoundError(
        "Failed to import Python submodule 'spaceros_procgen_envs._rs' that contains the Rust "
        "extension module. Please ensure that the package has been installed correctly."
    )

## If the simulation app is started, try to import the `spaceros_procgen_envs.tasks` submodule
## recursively to register all tasks
if environ.get("SPACEROS_DEMO_SKIP_REGISTRATION", "false").lower() in ["true", "1"]:
    print(
        f"INFO: [SPACEROS_DEMO_SKIP_REGISTRATION={environ.get('SPACEROS_DEMO_SKIP_REGISTRATION')}] Skipping "
        "the registration of Space ROS ProcGen Environments tasks."
    )
elif is_sim_app_started():
    import_modules_recursively(
        module_name=f"{__name__}.tasks",
        ignorelist=[
            "common",
        ],
    )
else:
    raise RuntimeError(
        "Tasks of the Space ROS ProcGen Environments cannot be registered because the simulation "
        "is not running. Please import the 'spaceros_procgen_envs' module after starting the "
        "Omniverse simulation app. Alternatively, set the 'SPACEROS_DEMO_SKIP_REGISTRATION' environment "
        "variable to 'true' to skip the registration of tasks."
    )
