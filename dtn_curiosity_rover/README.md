# Curiosity Rover over DTN (Bundle Protocol v7)

This demo drives the Curiosity rover from a "ground" node to a "rover" node over a
Delay/Disruption Tolerant Networking (DTN) link with a configurable light-time delay
and link outages. Drive commands travel as Bundle Protocol v7 (RFC 9171) bundles,
using NASA's cFS DTN node, [BPNode](https://github.com/nasa/bp) with
[bplib](https://github.com/nasa/bplib).

It builds on [`racs2_demos_on_spaceros`](../racs2_demos_on_spaceros), which connects
Space ROS to cFS through JAXA's [RACS2 bridge](https://github.com/jaxa/racs2_bridge).
Here the cFS side is split in two, with a DTN link in the middle:

```
 ground (cFS, ipn:100)                         rover (cFS, ipn:200)              rover (Space ROS)
 run_app -> BPNode  ==== bundles over UDP ===>  BPNode -> racs2_bridge_client --> bridge_py_s -> move_wheel -> Curiosity (Gazebo)
 keyboard            delay, outages (tc netem)                       websocket
```

What it shows:

- **Light-time delay:** each command reaches the rover one light time after it was sent
  (5 s by default; 1.3 s would be the Moon, 3 to 22 minutes Mars).
- **Store and forward:** commands sent while the link is down are not lost. BPNode's
  custody transfer retransmits each bundle until the rover node takes custody of it.
- **Driving with a delay:** the rover keeps executing the last command for a full light
  time after the operator changes it. With the default delay, a rover told to stop after
  driving forward rolls on for about 6 s more.

## Building the demo

This demo uses the images of [`racs2_demos_on_spaceros`](../racs2_demos_on_spaceros)
(Space ROS side of the rover) and [`curiosity_rover`](../curiosity_rover) (Gazebo), and
builds one more image for the two cFS DTN nodes. To build all of them:

```bash
git clone https://github.com/space-ros/demos.git
cd demos/dtn_curiosity_rover
./build.sh
```

## Running the demo

```bash
./run.sh
```

This starts Gazebo with the Curiosity rover, the Space ROS nodes, and the two DTN nodes,
then attaches your terminal to the ground node's cFS console. Drive the rover from there:

* `w`: forward
* `s`: backward
* `a`: turn left
* `d`: turn right
* `x`: stop

Detach from the console without stopping the demo with `Ctrl-P` `Ctrl-Q`. Stop the demo
with `docker compose down`.

Commands can also be sent from another terminal:

```bash
./drive.sh w
```

### Changing the link

The one-way delay is set with `LINK_DELAY_MS` when the demo starts, for example
`LINK_DELAY_MS=1300 ./run.sh` for the Moon. While the demo runs:

```bash
./link.sh delay 10000   # one-way light time in milliseconds
./link.sh down          # start an outage
./link.sh up            # end it
./link.sh status
```

### What you will see

The ground console prints each command as it is sent:

```
[ground 07:04:30.531Z] sent 'forward' (linear.x=2.0 angular.z=0.0) as MsgID 0x1EFE
```

and the rover node logs it when it arrives (`docker logs -f dtn_rover_cfs`):

```
[rover 07:04:36.518Z] received /cmd_vel command from the ground, forwarding to ROS 2
```

With the default 5 s delay, commands arrive about 6 s after they are sent (the light
time plus BPNode's processing). A command sent during an outage is delivered after the
link comes back, by the next custody retransmission (every 30 s). Because that
retransmission happens per bundle, a command that was lost can arrive after later ones.

## How it works

| Container | Role |
| --- | --- |
| `ground` | cFS node `ipn:100` with BPNode and `run_app`, which turns key presses into RACS2 user messages (topic `/cmd_vel`, protobuf `geometry_msgs/Twist`) on the Software Bus. |
| `rover_cfs` | cFS node `ipn:200` with BPNode and `racs2_bridge_client`, which forwards the messages BPNode delivers to the RACS2 ROS 2 bridge. |
| `rover_ros` | The `racs2_demos_on_spaceros` image, unchanged: `bridge_py_s` and the Curiosity demo nodes. |
| `curiosity_gui` | The `curiosity_rover` Gazebo simulation. |

The two cFS nodes share the `dtn_link` network; the rover containers share `rover_lan`.

On the ground node, BPNode's ADU proxy subscribes to the RACS2 message ID (`0x1EFE`) and
sends each message, header included, as the payload of a custodial bundle to
`ipn:200.64`. The rover's BPNode publishes the payload unchanged on its Software Bus,
where `racs2_bridge_client` picks it up. `start_node.sh` brings up each node's contact
with BPNode's `CONTACT_SETUP` and `CONTACT_START` commands, sent through `ci_lab`.

`dtn_link.sh` applies `tc netem` to IP packets on each node's `dtn_link` interface (ARP
is left unshaped so address resolution does not time out under long delays).

### cFS build

`Dockerfile.cfs` builds cFS `v7.0.0` with BPNode and bplib `v7.0.5` and CF `v7.0.0`.
The mission configuration starts from the cFE `sample_defs` and adds BPNode's per-node
tables and startup scripts from `nasa/bp` `dtn_defs`, with these changes for cFS
`v7.0.0`:

* BPNode's tables in `cfs/defs/cpu*/tables`: each node's contact 0 points at the other
  node, the ground's ADU proxy carries `0x1EFE`, and custody timers are shortened.
* Longer file, path and table names, and more Software Bus pipes, as in `dtn_defs`.
* The SCH_LAB and TO_LAB table names that cFS `v7.0.0` registers, and include paths for
  those tables.
* CF is built (BPNode's tables reference its message IDs) but not started.

`run_app` and `racs2_bridge_client` are cFS v7 ports of the apps in
`racs2_demos_on_spaceros` and `jaxa/racs2_bridge`, using the same message layout and
websocket frame format.

### Limits

The custody retransmit timeout is 30 s, so keep the one-way delay below about 14 s, or
retransmissions start before the custody signal can come back. For longer delays, raise
`RetransmitTimeout` in `cfs/defs/cpu*/tables/bpnode_contacts.c` above twice the delay.

## References

* [BPNode](https://github.com/nasa/bp) and [bplib](https://github.com/nasa/bplib), NASA (Apache-2.0)
* [RACS2 bridge](https://github.com/jaxa/racs2_bridge), JAXA (Apache-2.0)
* [RFC 9171: Bundle Protocol Version 7](https://www.rfc-editor.org/rfc/rfc9171.html)
