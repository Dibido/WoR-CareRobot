# Visualization node

This node listens to topics specified in the launch file. This data is then modifief if needed and then published on another topic. RViz is configured to display data that is published on this topic.


## Build

```
$ cd <YOUR_CATKIN_WORKSPACE_PATH>
$ catkin_make
$ source devel/setup.bash
```


## Run

Launch the `sim_world` package first. You can read more about this launch file in the README.md file of the `sim_world` package

```
$ roslaunch sim_world world.launch robot:=franka
```

Then launch the `vis_control` node.

```
$ roslaunch vis_control visualizer.launch view:=all
```

There are a few different parameter options for this launch file, there will only be data visualized once the matching requirements are matched.

| Parameter             | Description                        | Requirements                                       |
|-----------------------|------------------------------------|----------------------------------------------------|
| `view:=all` (default) | Visualize all available data       | Match all preconditions in this table              |
| `view:=kinect`        | Visualize only the kinect data     | `$ roslaunch sim_world world.launch`               |
| `view:=lidar`         | Visualize only the lidar data      | `$ roslaunch sim_world world.launch`               |
| `view:=path`          | Visualize only the path data       | `$ roslaunch <PATH_PLANNER>`                       |
| `view:=viewObjects`   | Visualize only the ViewObject data | `$ roslaunch vision vision.launch`                 |


## Known problems

* The DetectedObjectVisualizer contains a pretty large construction to parse the DetectedObjects message that is received from the vision component. This message has very poor design since every object has data seperated in multiple lists. Ideally the default ROS message "visualization_msgs/MarkerArray.h" will be received from the vision component in the future because it has much better design and contains the exact same data as the custom DetectedObjects message.
