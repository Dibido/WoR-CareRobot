Application will connect with the hardware lidar, and will publish all measurements on topic /sensor/lidar. These measurements form the input for the lidar_application, for more information see the README.md in lidar_application package.

After running catkin_make, application can be run by "rosrun rplidar_hardware node". Optionally you can add the serial port as argument, default value is ttyUSB0. So if lidar is attached to ttyUSB1: "rosrun rplidar_hardware node ttyUSB1".
