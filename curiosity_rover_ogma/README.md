# Generating runtime monitors for the Curiosity Rover

This directory contains a project that can be used with
[Ogma](https://github.com/nasa/ogma) to generate a runtime monitor for the
Curiosity Rover demo included with Space ROS.

In particular, it helps quickly create nodes that detect if certain
properties are violated.

For the purposes of illustrating how to use this tool, the project included in
this directory checks if the absolute value of the rover's X coordinate in the
odometry's pose becomes greater than 2.5. When that happens, the generated node
starts sending out violation messages.

The property in question is listed in the file `document.json`, which lists the
following property, where `input_signal` represents the rover's X coordinate:

```
{ "id":          "KeepRoverInCheck",
  "formula":     "abs input_signal <= 2.5",
  "description": "The rover stays near the starting point"
}
```

The rest of this README explains how to compile this property into a ROS 2 node
that will run alongside the Curiosity rover demo and check its position at all
times.

# Compilation

To compile everything, run, from this specific demo's directory:

```sh
./build.sh
```

# Execution

To run the demo, we need at least 4 terminals.

## Terminal 1

Start the curiosity rover demo from its own directory:

```sh
cd ../curiosity_rover
./run.sh
```

## Terminal 2

Start the monitoring node from this specific demo's directory.

```sh
./run.sh
```

Once the container starts, run:

```sh
$ source install/setup.bash
$ ros2 run copilot copilot
```

That node does not print any output.

## Terminal 3

Log into this demo's container and listen for runtime monitoring
violations:

```sh
$ docker exec -it osrf_space-ros-curiosity-rover-ogma /bin/bash
$ source install/setup.bash
$ ros2 topic echo /copilot/handlerKeepRoverInCheck \
    | while IFS= read -r line; do \
        echo "$(date '+%Y-%m-%d %H:%M:%S') $line"; \
      done
```

## Terminal 4

Instruct the rover to move forward by executing a call from the Curiosity demo
container:

```sh
$ docker exec -it curiosity_rover-curiosity_demo-1 /bin/bash
$ ros2 service call /move_forward std_srvs/srv/Empty
```

After approximately 1 minute (when the absolute value of the X coordinate of
the rover's odometry's pose becomes greater than 2.5), Terminal 3 will start
printing messages like the following:

```
2026-09-03 13:34:26 {}
2026-09-03 13:34:26 ---
2026-09-03 13:34:27 {}
2026-09-03 13:34:27 ---
2026-09-03 13:34:28 {}
2026-09-03 13:34:28 ---
2026-09-03 13:34:28 {}
2026-09-03 13:34:28 ---
2026-09-03 13:34:29 {}
2026-09-03 13:34:29 ---
```

Each of those indicates a violation has occurred, meaning that the rover has
left the "safe" zone. If you turn the rover around to get closer to the
starting point (where the absolute value of the X coordinate becomes less than
2.5), the messages will stop, and will resume if the rover distances itself
from the starting point again.
