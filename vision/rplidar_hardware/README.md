Application will connect with the hardware lidar, and will publish all measurements on topic /sensor/lidar. These measurements form the input for the lidar_application, for more information see the README.md in lidar_application package.

After running catkin_make, application can be run by "rosrun rplidar_hardware node". Optionally you can add the serial port as argument, default value is ttyUSB0. So if lidar is attached to ttyUSB1: "rosrun rplidar_hardware node ttyUSB1".

NOTE: For (at the moment) unknown reasons, this package might fail to build when first pulled from git. This problem can be fixed by executing the following steps:
1. Download zip from https://github.com/slamtec/rplidar_ros
2. Move folder 'sdk' from the downloaded zip, into the rplidar_hardware package. Overwrite the existing sdk folder.
Now the package will succesfully build with catkin_make. The cause of this issue will be investigated later.