# Simulation

Dit component is bedoeld om de werkelijkheid te simuleren. Dit wordt gedaan met het programma Gazebo.

## Packages

| Package               | Description                           |
|-----------------------|---------------------------------------|
| sim_agv               | Plugin voor het AGV model             |
| sim_cup               | Plugin voor het beker model           |
| sim_kinect            | Plugin voor het kinect model          |
| sim_lidar             | Plugin voor het lidar model           |
| sim_robot             | Plugin voor het cup model             |
| sim_sonar             | Plugin voor het sonar model           |
| [gazebo_grasp_plugin] | Plugin which helps grasping in Gazebo |

Tot slot is er de `sim_world` package. In deze package staan alle modellen en werelden. Het `world.launch` bestand in deze package kan worden gebruikt om een wereld met verschillende modellen in Gazebo op te starten.
te 

[gazebo_grasp_plugin]: https://github.com/JenniferBuehler/gazebo-pkgs