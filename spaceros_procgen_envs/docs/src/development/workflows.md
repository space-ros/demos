# Development Workflows

The `spaceros_procgen_envs` package is based around using Docker, which in itself provides an isolated development environments. Using the `run.sh` script already mounts the source code of the demo into the container, which in itself already makes the Docker setup quqite convenient for development. However, the following workflows are also available.

## Dev Containers

Dev Containers allow for a fully isolated development environment tailored to specific project needs. This is particularly useful for ensuring all dependencies are installed and consistent across different development machines.

1. **Dev Containers Introduction**:

   - The repository includes preconfigured settings for [Dev Containers](https://containers.dev), which are development environments hosted within Docker containers. These containers make it easy to manage dependencies and ensure that the development environment is fully consistent across various setups.

1. **Modifying the Dev Container**:

   - You can customize the included `.devcontainer/devcontainer.json` file to suit specific project or development needs. This configuration file controls aspects such as base images, development tools, extensions, and environment variables within the container.

1. **Opening the Repository in a Dev Container**:

   - To simplify the process of building and opening the repository as a Dev Container in Visual Studio Code (VS Code), an automated script is available. Running the following command will set up the containerized environment and open it in VS Code:

   ```bash
   spaceros_procgen_envs/.devcontainer/open.sh
   ```
