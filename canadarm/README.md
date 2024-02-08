# Canadarm


(This is a temp instruction before the reorg)

Build and source the ```canadarm``` package:

```
ros2 launch canadarm canadarm.launch.py
```

To move the arm to an outstretched pose:

```
ros2 service call /open_arm std_srvs/srv/Empty
```

To move the arm to its default (close) pose of all zeros:

```
ros2 service call /close_arm std_srvs/srv/Empty
```

To move the arm to a random pose:

```
ros2 service call /random_arm std_srvs/srv/Empty
```
