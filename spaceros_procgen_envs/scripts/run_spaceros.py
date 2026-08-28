#!/home/spaceros-user/isaac-sim/python.sh
"""
Entrypoint script for showcasing Space ROS ProcGen Environments

Examples:
    ros2 run spaceros_procgen_envs run_spaceros.py
    ros2 run spaceros_procgen_envs run_spaceros.py --task sample_collection --num_envs 4
"""

from _cli_utils import add_default_cli_args, argparse, launch_app, shutdown_app
from omni.isaac.lab.app import AppLauncher


def main(launcher: AppLauncher, args: argparse.Namespace):
    ## Note: Importing modules here due to delayed Omniverse Kit extension loading
    from os import path

    import gymnasium
    import torch
    from omni.isaac.kit import SimulationApp
    from omni.isaac.lab.utils.dict import print_dict

    import spaceros_procgen_envs  # Noqa: F401
    from spaceros_procgen_envs.core.interfaces import SpaceROS
    from spaceros_procgen_envs.utils.parsing import create_logdir_path, parse_task_cfg

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

    ## Initialize the environment
    observation, info = env.reset()

    ## Create Space ROS interface
    spaceros_interface = SpaceROS(env)

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

    ## Run the environment with Space ROS interface
    with torch.inference_mode():
        while sim_app.is_running():
            # Get actions from Space ROS
            actions = spaceros_interface.actions

            # Step the environment
            observation, reward, terminated, truncated, info = env.step(actions)

            # Publish to Space ROS
            spaceros_interface.publish(observation, reward, terminated, truncated, info)

            # Process requests from Space ROS
            spaceros_interface.update()

            # Note: Each environment is automatically reset (independently) when terminated or truncated

    ## Close the environment
    env.close()


### Helper functions ###
def parse_cli_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
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
