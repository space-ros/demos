## Adding New Environments

To introduce a new environment into the `spaceros_procgen_envs` package, the process is straightforward and modular. Here’s a step-by-step guide:

1. **Duplicate an Existing Environment**:

   - Navigate to the `spaceros_procgen_envs/tasks` directory, which houses the existing environments.
   - Copy one of the existing demo or task folders (e.g., `sample_collection` or one that resembles your desired task/demo more) and rename it to the name of your new environment.

1. **Modify the Environment Configuration**:

   - After duplicating, customize your new environment by altering the configuration files and task implementation code within the folder. This may include asset selection, interaction rules, or specific environmental dynamics.
   - Any changes can be made to the environment’s configuration, asset generation, or task-specific code depending on the desired functionality and behavior.

1. **Automatic Registration**:

   - The new environment will be automatically detected by the `spaceros_procgen_envs` package due to its presence in the `tasks` directory. The environment will be registered under the directory name you assigned during the duplication process.
   - There is no need for manual registration, as the system dynamically loads environments based on the folder structure.

1. **Running Your New Environment**:

   - To run or test the new environment, you can invoke it using the same commands as existing environments, specifying the name of your new environment (task) in the `--task` argument.
