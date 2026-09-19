#!/home/spaceros-user/isaac-sim/python.sh
"""
Native teleoperation with Space ROS ProcGen Environments inside the Isaac Sim window

Examples:
    ros2 run spaceros_procgen_envs teleop.py
    ros2 run spaceros_procgen_envs teleop.py --task sample_collection --num_envs 4 --teleop_device spacemouse
"""

from _cli_utils import add_default_cli_args, argparse, launch_app, shutdown_app
from omni.isaac.lab.app import AppLauncher

CONTROL_SCHEME_MANIPULATION_KEYBOARD = """
    +------------------------------------------------+
    |  Keyboard Scheme (focus the Isaac Sim window)  |
    +------------------------------------------------+
    +------------------------------------------------+
    | Toggle Gripper: [K]   | Reset: [ L ]           |
    +------------------------------------------------+
    | Translation                                    |
    |             [ W ] (+X)            [ Q ] (+Z)   |
    |               ↑                     ↑          |
    |               |                     |          |
    |  (-Y) [ A ] ← + → [ D ] (+Y)        +          |
    |               |                     |          |
    |               ↓                     ↓          |
    |             [ S ] (-X)            [ E ] (-Z)   |
    |------------------------------------------------|
    | Rotation                                       |
    |       [ Z ] ←--------(±X)--------→ [ X ]       |
    |                                                |
    |       [ T ] ↻--------(±Y)--------↺ [ G ]       |
    |                                                |
    |       [ C ] ↺--------(±Z)--------↻ [ V ]       |
    +------------------------------------------------+
"""
CONTROL_SCHEME_MANIPULATION_INVERT_VIEW = True

CONTROL_SCHEME_AERIAL_ROBOTICS_KEYBOARD = """
    +------------------------------------------------+
    |  Keyboard Scheme (focus the Isaac Sim window)  |
    +------------------------------------------------+
    +------------------------------------------------+
    | Reset: [ L ]                                   |
    +------------------------------------------------+
    |                  Translation                   |
    |             [ W ] (+X)            [ Q ] (+Z)   |
    |               ↑                     ↑          |
    |               |                     |          |
    |  (-Y) [ A ] ← + → [ D ] (+Y)        +          |
    |               |                     |          |
    |               ↓                     ↓          |
    |             [ S ] (-X)            [ E ] (-Z)   |
    |------------------------------------------------|
    |                    Rotation                    |
    |       [ C ] ↺--------(±Z)--------↻ [ V ]       |
    +------------------------------------------------+
"""

CONTROL_SCHEME_MOBILE_ROBOTICS_KEYBOARD = """
    +------------------------------------------------+
    |  Keyboard Scheme (focus the Isaac Sim window)  |
    +------------------------------------------------+
    +------------------------------------------------+
    | Reset: [ L ]                                   |
    +------------------------------------------------+
    | Planar Motion                                  |
    |                     [ W ] (+X)                 |
    |                       ↑                        |
    |                       |                        |
    |          (-Y) [ A ] ← + → [ D ] (+Y)           |
    |                       |                        |
    |                       ↓                        |
    |                     [ S ] (-X)                 |
    +------------------------------------------------+
"""


def main(launcher: AppLauncher, args: argparse.Namespace):
    ## Note: Importing modules here due to delayed Omniverse Kit extension loading
    from os import path

    import gymnasium
    import numpy as np
    import torch
    from omni.isaac.kit import SimulationApp
    from omni.isaac.lab.devices import Se3Gamepad, Se3Keyboard, Se3SpaceMouse
    from omni.isaac.lab.utils.dict import print_dict

    import spaceros_procgen_envs  # Noqa: F401
    from spaceros_procgen_envs.core.actions import (
        ManipulatorTaskSpaceActionCfg,
        MultiCopterActionGroupCfg,
        WheeledRoverActionGroupCfg,
    )
    from spaceros_procgen_envs.utils.parsing import create_logdir_path, parse_task_cfg

    if args.headless:
        raise ValueError("Native teleoperation is only supported in GUI mode.")

    ## Extract simulation app
    sim_app: SimulationApp = launcher.app

    # Parse configuration
    task_cfg = parse_task_cfg(
        task_name=args.task,
        device=args.device,
        num_envs=args.num_envs,
        use_fabric=not args.disable_fabric,
    )
    # Disable truncation
    if hasattr(task_cfg, "enable_truncation"):
        task_cfg.enable_truncation = False

    ## Create the environment
    env = gymnasium.make(
        id=args.task, cfg=task_cfg, render_mode="rgb_array" if args.video else None
    )

    ## Create controller
    if args.teleop_device.lower() == "keyboard":
        teleop_interface = Se3Keyboard(
            pos_sensitivity=0.05 * args.pos_sensitivity,
            rot_sensitivity=0.05 * args.rot_sensitivity,
        )
    elif args.teleop_device.lower() == "spacemouse":
        teleop_interface = Se3SpaceMouse(
            pos_sensitivity=0.05 * args.pos_sensitivity,
            rot_sensitivity=0.005 * args.rot_sensitivity,
        )
    elif args.teleop_device.lower() == "gamepad":
        teleop_interface = Se3Gamepad(
            pos_sensitivity=0.1 * args.pos_sensitivity,
            rot_sensitivity=0.1 * args.rot_sensitivity,
        )
    else:
        raise ValueError(f"Invalid device interface '{args.teleop_device}'.")
    teleop_interface.reset()
    teleop_interface.add_callback("L", env.reset)
    print(teleop_interface)

    ## Initialize the environment
    observation, info = env.reset()

    ## Add wrapper for video recording (if enabled)
    if args.video:
        logdir = create_logdir_path("spaceros", args.task)
        video_kwargs = {
            "video_folder": path.join(logdir, "videos"),
            "step_trigger": lambda step: step % args.video_interval == 0,
            "video_length": args.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gymnasium.wrappers.RecordVideo(env, **video_kwargs)

    def process_actions(twist: np.ndarray, gripper_cmd: bool) -> torch.Tensor:
        twist = torch.tensor(
            twist, dtype=torch.float32, device=env.unwrapped.device
        ).repeat(env.unwrapped.num_envs, 1)
        if isinstance(env.unwrapped.cfg.actions, ManipulatorTaskSpaceActionCfg):
            if CONTROL_SCHEME_MANIPULATION_INVERT_VIEW:
                twist[:, :2] *= -1.0
            gripper_action = torch.zeros(twist.shape[0], 1, device=twist.device)
            gripper_action[:] = -1.0 if gripper_cmd else 1.0
            return torch.concat([twist, gripper_action], dim=1)
        elif isinstance(env.unwrapped.cfg.actions, MultiCopterActionGroupCfg):
            return torch.concat(
                [
                    twist[:, :3],
                    twist[:, 5].unsqueeze(1),
                ],
                dim=1,
            )

        elif isinstance(env.unwrapped.cfg.actions, WheeledRoverActionGroupCfg):
            return twist[:, 0:2]

    # Print the appropriate control scheme for keyboard-based teleoperation
    if args.teleop_device.lower() == "keyboard":
        if isinstance(env.unwrapped.cfg.actions, ManipulatorTaskSpaceActionCfg):
            print(CONTROL_SCHEME_MANIPULATION_KEYBOARD)
        elif isinstance(env.unwrapped.cfg.actions, MultiCopterActionGroupCfg):
            print(CONTROL_SCHEME_AERIAL_ROBOTICS_KEYBOARD)
        elif isinstance(env.unwrapped.cfg.actions, WheeledRoverActionGroupCfg):
            print(CONTROL_SCHEME_MOBILE_ROBOTICS_KEYBOARD)

    ## Run the environment
    with torch.inference_mode():
        while sim_app.is_running():
            # Get actions from the teleoperation interface
            actions = process_actions(*teleop_interface.advance())

            # Step the environment
            observation, reward, terminated, truncated, info = env.step(actions)

            # Note: Each environment is automatically reset (independently) when terminated or truncated

    ## Close the environment
    env.close()


### Helper functions ###
def parse_cli_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    algorithm_group = parser.add_argument_group(
        "teleop",
        description="Arguments for teleoperation",
    )
    algorithm_group.add_argument(
        "--teleop_device",
        type=str,
        default="keyboard",
        help="Device for interacting with environment",
    )
    algorithm_group.add_argument(
        "--pos_sensitivity",
        type=float,
        default=15.0,
        help="Sensitivity factor for translation.",
    )
    algorithm_group.add_argument(
        "--rot_sensitivity",
        type=float,
        default=10.0,
        help="Sensitivity factor for rotation.",
    )
    add_default_cli_args(parser)
    return parser.parse_args()


if __name__ == "__main__":
    # Parse arguments
    args = parse_cli_args()

    # Launch the app
    launcher = launch_app(args)

    # Run the main function
    main(launcher=launcher, args=args)

    # Shutdown the app
    shutdown_app(launcher)
