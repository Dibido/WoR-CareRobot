# Simulation Lidar

Met deze package wordt de lidar gesimuleerd. De package bestaat uit een library plugin die 
gebruikt wordt in de lidar model.

## Build

Build de plugin met catkin
* `catkin_make sim_lidar_plugin`

## Run

Om de lidar te testen in een wereld:
* `roslaunch sim_lidar lidar_plugin.launch` 

Launch file arguments
* Geen

### Data

Data op topics:
* /sensor/lidar/scan
    * laser scan data van de lidar

## Test

### User test

* Confluence pagina: http://wor.wiki.icaprojecten.nl/confluence/display/UCOSAX/User+test+-+Lidar+plugin

## Current problems

* Geen