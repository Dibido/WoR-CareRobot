# Simulation robot

This package is used to simulate the franka panda model plugin.


## Build

Build plugin 
* `catkin_make sim_robot_plugin`

## Run
execute the following to simulate the Franka Panda:

* `roslaunch sim_robot franka.launch`
    * World with only the Franka Panda.

Launch file arguments
* none


## Test

### build unittests

1. Build  `catkin_make sim_robot_unit_tests`.
2. Run  `rosrun sim_robot sim_robot_unit_tests`.

## Current problems

* none