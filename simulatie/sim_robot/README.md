# Simulation robot

Met deze package worden de robotarmen gesimuleerd. De package bestaat uit een library plugin die 
gebruikt wordt in de zowel de al5d model als de franka model.

## Build

Build de plugin met catkin
* `catkin_make sim_robot_plugin`

## Run

Om de robots te testen in een wereld:
* `roslaunch sim_robot al5d.launch`
    * Een wereld met alleen een al5d robotarm
* `roslaunch sim_robot franka.launch`
    * Een wereld met alleen een franka panda robotarm

Launch file arguments
* Geen


## Test

Het testen van deze package kan doormiddel van de volgende commando's

### Parser test

1. Build de test met `catkin_make sim_robot_parser_test`.
2. Run de tests met `rosrun sim_robot sim_robot_parser_test`.

### Joint controller test

1. Build de test met `catkin_make sim_robot_joint_test`.
2. Run de tests met `rosrun sim_robot sim_robot_joint_test`.

## Current problems

* Geen