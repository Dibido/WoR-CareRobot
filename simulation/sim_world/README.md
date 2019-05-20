# sim_world package

This package contains all model and world files.


## Build

```
$ cd <YOUR_CATKIN_WORKSPACE_PATH>
$ catkin_make
$ source devel/setup.bash
```


## Run

Launch the `sim_world` node.

```
roslaunch sim_world world:=franka
```

There are a few different parameter options for this launch file.

| Parameter                     | World description                     | Used to
|-------------------------------|---------------------------------------|-------------------------|
| `world:=franka_agv` (default) | Franka arm, sensors, tables, agv, cup | Simulate the full setup |
| `world:=franka_empty`         | Franka arm                            | Mirror the real world   |
| `world:=franka`               | Franka arm, sensors, table, cup       | -                       |
| `world:=al5d`                 | AL5D arm                              | -                       |
| `world:=current_world`        | Current world                         | -                       |
    

## Known problems

* Fluid spilling only looks at the cup orientation and not at acceleration
* A cup falls of a AGV vechicle if: it is placed on the edge, if the AGV drives to fast, if the cup is placed to high above the AGV, if the the AGV accelerates to fast.
