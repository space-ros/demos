# Cleaning the Assets Cache

After running several demos or simulations, the procedurally generated assets (such as textures and meshes) can accumulate in the cache. To free up disk space, you can clean this cache:

```bash
spaceros_procgen_envs/run.sh ros2 run spaceros_procgen_envs clean_procgen_cache.py
```

If you're also using Docker and wish to remove the corresponding image, run:

```bash
docker rmi openrobotics/spaceros_procgen_envs
```
