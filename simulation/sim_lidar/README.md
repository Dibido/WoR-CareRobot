# Simulation Lidar
This package contains a lidar model and a library plugin.


# Design

http://wor.wiki.icaprojecten.nl/confluence/display/EBGURG/DDD+-+Design+LIDAR

## Build

Build the plugin with catkin:
* `catkin_make sim_lidar_plugin`

## Run

Execute:
* `roslaunch sim_lidar lidar_plugin.launch` 

Launch file arguments
* None

### Data

Data being published on the following topics:
* /sensor/lidar/scan
    * laser scan data lidar
* /sensor/lidar
    * scan data LidarData message

## Test

### build unittests

1. Build  `catkin_make sim_lidar_unit_tests`.
2. Run  `rosrun sim_robot sim_lidar_unit_tests`.

## Current problems
* None