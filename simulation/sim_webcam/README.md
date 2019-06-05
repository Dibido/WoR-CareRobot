# Simulation webcam

This package contains a webcam model and a library plugin.


## Build

Build the plugin with catkin
* `catkin_make sim_webcam_plugin`

## Run

execute:
* `roslaunch sim_webcam launch.launch` 

Launch file arguments
* none

### Data

Data being published on the following topics:
* /sensor/webcam/raw_img
    * video data van de vga camera

## Test

There are currently no unit tests available.

## Current problems

* Noise is yet to be implemented.
