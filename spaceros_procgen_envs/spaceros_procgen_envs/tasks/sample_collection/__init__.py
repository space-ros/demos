from spaceros_procgen_envs.utils.registry import register_tasks

from .task import Task, TaskCfg
from .task_visual import VisualTask, VisualTaskCfg

BASE_TASK_NAME = __name__.split(".")[-1]
register_tasks(
    {
        BASE_TASK_NAME: {},
        f"{BASE_TASK_NAME}_visual": {
            "entry_point": VisualTask,
            "task_cfg": VisualTaskCfg,
        },
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)
