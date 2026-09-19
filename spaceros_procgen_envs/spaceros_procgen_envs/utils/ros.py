from importlib.util import find_spec
from os import environ

import omni.ext


def enable_ros2_bridge():
    if find_spec("rclpy") is not None:
        # No-op if rclpy can already be imported, which means that Space ROS is already sourced
        return

    ld_library_path = environ.get("LD_LIBRARY_PATH", None)
    ld_library_path = f":{ld_library_path}" if ld_library_path else ""
    environ["LD_LIBRARY_PATH"] = (
        f"{omni.kit.paths.get_omni_path()}/exts/omni.isaac.ros2_bridge/humble/lib{ld_library_path}"
    )

    # Get the extension manager and list of available extensions
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    extensions = extension_manager.get_extensions()

    # Extract the ROS extension
    ros_extension = [ext for ext in extensions if "ros2_bridge" in ext["id"]][0]

    # Load the ROS extension if it is not already loaded
    if not extension_manager.is_extension_enabled(ros_extension["id"]):
        extension_manager.set_extension_enabled_immediate(ros_extension["id"], True)
