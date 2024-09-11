#!/home/spaceros-user/isaac-sim/python.sh
"""
Utility script for listing all registered Space ROS ProcGen Environments

Usage:
    ros2 run spaceros_procgen_envs list_envs.py
"""

from _cli_utils import add_default_cli_args, argparse, launch_app, shutdown_app


def main():
    import gymnasium
    from prettytable import PrettyTable

    import spaceros_procgen_envs  # noqa: F401
    from spaceros_procgen_envs.utils.registry import get_spaceros_tasks

    # Table config
    table = PrettyTable(["id (task/demo)", "entrypoint", "config"])
    table.title = "Space ROS ProcGen Eenvironments"
    table.align = "l"

    ## Fill the table with "spaceros" tasks
    for task_id in get_spaceros_tasks():
        spec = gymnasium.registry[task_id]
        table.add_row(
            [
                task_id,
                str(spec.entry_point),
                (
                    str(spec.kwargs["task_cfg"])
                    .removeprefix("<class '")
                    .removesuffix("'>")
                    .replace(spec.entry_point.split(":")[0], "$MOD")
                ),
            ]
        )

    ## Print the results
    print(table)


### Helper functions ###
def parse_cli_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    add_default_cli_args(parser)
    return parser.parse_args()


if __name__ == "__main__":
    # Parse arguments
    args = parse_cli_args()

    # Set headless mode
    args.headless = True

    # Launch the app
    launcher = launch_app(args)

    # Run the main function
    main()

    # Shutdown the app
    shutdown_app(launcher)
