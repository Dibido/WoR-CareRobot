# Simulation sonar

Met deze package wordt de sonar sensor gesimuleerd in de Gazebo wereld. De package bestaat uit een library plugin die
gebruikt wordt in het sonar model.

## Build

Build de plugin met catkin
* `catkin_make --pkg=sim_world`
* `catkin_make --pkg=sim_sonar`

## Run

Om de sonar sensor te testen in een wereld:
* `roslaunch sim_sonar launch.launch`
