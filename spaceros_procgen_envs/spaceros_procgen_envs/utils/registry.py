from typing import Any, Dict, List, Literal, Optional, Union

import gymnasium


def register_tasks(
    tasks: Dict[
        str,
        Dict[
            Literal["entry_point", "task_cfg", "cfg_dir"],
            Union[gymnasium.Env, Any, str],
        ],
    ],
    *,
    default_entry_point: Optional[gymnasium.Env] = None,
    default_task_cfg: Optional[Any] = None,
    namespace: str = "spaceros",
):
    for id, cfg in tasks.items():
        entry_point = cfg.get("entry_point", default_entry_point)
        gymnasium.register(
            id=f"{namespace}/{id}",
            entry_point=f"{entry_point.__module__}:{entry_point.__name__}",
            kwargs={
                "task_cfg": cfg.get("task_cfg", default_task_cfg),
            },
            disable_env_checker=True,
        )


def get_spaceros_tasks() -> List[str]:
    return [env_id for env_id in gymnasium.registry.keys() if "spaceros" in env_id]
