import argparse
import subprocess
from os import environ, path

from omni.isaac.lab.app import AppLauncher


def add_default_cli_args(parser: argparse.Namespace):
    # Environment
    class _AutoNamespaceTaskAction(argparse.Action):
        def __call__(self, parser, namespace, values, option_string=None):
            if "/" not in values:
                DEFAULT_TASK_NAMESPACE: str = "spaceros"
                values = f"{DEFAULT_TASK_NAMESPACE}/{values}"
            setattr(namespace, self.dest, values)

    environment_group = parser.add_argument_group(
        "environment arguments",
        description="Arguments for environment.",
    )
    environment_group.add_argument(
        "--task",
        "--demo",
        "--env",
        type=str,
        default="spaceros/perseverance",
        action=_AutoNamespaceTaskAction,
        help="Name of the task/demo/env. You can run the `list_envs.py` script to get a list of all registered tasks.",
    )
    environment_group.add_argument(
        "--seed", type=int, default=None, help="Seed used for the environment"
    )
    environment_group.add_argument(
        "--num_envs",
        type=int,
        default=1,
        help="Number of environments to simulate in parallel.",
    )

    # Compute
    compute_group = parser.add_argument_group(
        "compute arguments",
        description="Arguments for compute.",
    )
    compute_group.add_argument(
        "--disable_fabric",
        action="store_true",
        default=False,
        help="Disable fabric and use USD I/O operations.",
    )

    # Video recording
    video_recording_group = parser.add_argument_group(
        "video_recording arguments",
        description="Arguments for video_recording.",
    )
    video_recording_group.add_argument(
        "--video",
        action="store_true",
        default=False,
        help="Record videos.",
    )
    video_recording_group.add_argument(
        "--video_length",
        type=int,
        default=1000,
        help="Length of the recorded video (in steps).",
    )
    video_recording_group.add_argument(
        "--video_interval",
        type=int,
        default=20000,
        help="Interval between video recordings (in steps).",
    )

    # Experience
    experience_group = parser.add_argument_group(
        "experience arguments",
        description="Arguments for experience.",
    )
    experience_group.add_argument(
        "--disable_ui",
        action="store_true",
        default=False,
        help="Disable most of the Isaac Sim UI and set it to fullscreen.",
    )

    # Append app launcher arguments
    AppLauncher.add_app_launcher_args(parser)


def launch_app(args: argparse.Namespace) -> AppLauncher:
    _update_extension_module()
    _autoenable_cameras(args)
    _autoselect_experience(args)

    launcher = AppLauncher(launcher_args=args)

    if args.disable_ui:
        _disable_ui()

    return launcher


def shutdown_app(launcher: AppLauncher):
    launcher.app.close()


def _update_extension_module():
    subprocess.run(
        [
            environ.get("ISAAC_SIM_PYTHON", "python3"),
            "-m",
            "pip",
            "install",
            "--disable-pip-version-check",
            "--no-deps",
            "--no-compile",
            "--editable",
            path.dirname(path.dirname(path.realpath(__file__))),
        ],
    )


def _autoenable_cameras(args: argparse.Namespace):
    if not args.enable_cameras and (args.video or "visual" in args.task):
        args.enable_cameras = True


def _autoselect_experience(args: argparse.Namespace):
    ## Get relative path to the experience
    demo_dir = path.dirname(path.dirname(path.realpath(__file__)))
    experience_dir = path.join(demo_dir, "apps")

    ## Select the experience based on args
    experience = "spaceros"
    if args.headless:
        experience += ".headless"
    if args.enable_cameras:
        experience += ".rendering"
    experience += ".kit"

    ## Set the experience
    args.experience = path.join(experience_dir, experience)


def _disable_ui():
    import carb.settings

    settings = carb.settings.get_settings()
    settings.set("/app/window/hideUi", True)
    settings.set("/app/window/fullscreen", True)
