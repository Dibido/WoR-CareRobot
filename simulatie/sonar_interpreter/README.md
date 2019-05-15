
# Sonar Interpreter

# Launch the launch file with:
'roslaunch sonar_interpreter sonar_interpreter.launch'

# Install the following
'sudo apt install ros-melodic-rosserial' 
(See README.md body/sen_sonar)

# Launch the sen_sonar node
'roslaunch sen_sonar launch.launch port:=/dev/ttyACM{X}'

# To see the result check
'rostopic echo /object_speed'

# Tests
Unit tests are not included. Testing will be covered by test cases.

# Description
This node checks two ultrasonic sensors, a moving objects needs to pass past sensor1 before sensor2. 
It then calculates the speed based on the timestamp difference and the set location of the two sensors.
Sensor2 needs to be the sensor closest to 0,0/the robotarm.

# Confluence
http://wor.wiki.icaprojecten.nl/confluence/display/UCOSAX/Subsystem+Body+-+Sonar+interpreter
