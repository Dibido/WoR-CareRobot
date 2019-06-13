# Simulation Kinect
This package contains a kinect model and a library plugin.


## Build

Build the plugin with catkin
* `catkin_make sim_kinect_plugin`

## Run

execute:
* `roslaunch sim_kinect launch.launch` 

Launch file arguments
* none

### Data

Data being published on the following topics:
* /sensor/kinect/raw_img
    * video data van de vga camera
* /sensor/kinect/points
    * pointcloud data van de depth camera

## Test

There are currently no unit tests available.

## Current problems

* Noise is yet to be implemented.
