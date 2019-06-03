# Agv Parser
Parses the AGV position message recieved from the AGV gateway to a ROS topic.
## Pre-condition:
No data is being recieved from the AGV.
## Post-condition:
An AGV message is recieved through the terminal in the folowing format : "#S#[value]", example : "#S#0.22573".
An AGVSpeed message is output on the /sensor/agv topic.