# Simulation Cup

This package contains a cup model and a library plugin.

## Build

Build the plugin with catkin
* `catkin_make sim_cup_plugin`

## Run

execute:
* `roslaunch sim_cup just_a_cup.launch` 

Launch file arguments
* None

### Data

Data being published on the following topics:
* /sensor/kinect/raw_img
    * video data van de vga camera
* /sensor/kinect/points
    * pointcloud data van de depth camera

## Test

There are currently no unit tests available.

## Current problems

* None


