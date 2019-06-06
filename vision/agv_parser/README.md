# Agv Parser
Parses the AGV position message received from the AGV gateway to a ROS topic.
Can be started with any Serial port by doing the following:  
rosrun agv_parser agv_parser_main /dev/ttyUSB2
When the serial port is not given it tries to open /dev/ttyUSB1
## Pre-condition:
No data is being received from the AGV.
## Post-condition:
A AGV message is received through the terminal in the following format : "#S#[value]", example : "#S#0.22573".
The Message has been parsed and published on the /sensor/agv topic.
## Known issues
* The serial handler is a blocking while loop, could be changed to an asynchronous version with callbacks.
* The program crashes when the given serial port is not connected.