# Simulation Kinect

Met deze package wordt de kinect gesimuleerd. De package bestaat uit een library plugin die 
gebruikt wordt in de kinect model.

## Build

Build de plugin met catkin
* `catkin_make sim_webcam_plugin`

## Run

Om de kinect te testen in een wereld:
* `roslaunch sim_webcam launch.launch` 

Launch file arguments
* Geen

### Data

Data op topics:
* /sensor/webcam/raw_img
    * video data van de vga camera

## Test

Er zijn geen unit tests voor deze plugin.

## Current problems

* Geen
