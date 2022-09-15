## Dashboard disposition demo

To run the demo using earthly:

Set up ROS 2 sources
```
earthly +sources
```

Add the [`process_sarif`](https://github.com/space-ros/process_sarif) project.
```
git clone https://github.com/space-ros/process_sarif spaceros_ws/src/process_sarif -b build-results-archive
```

Run a build and test

```
earthly +test
```

TBC
