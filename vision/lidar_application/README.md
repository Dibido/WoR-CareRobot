After running catkin_make, lidar application can be run by 'rosrun lidar_application lidar_application_main'
After running catkin_make tests, unittests can be run by 'rosrun lidar_application lidar_application_unit_tests'

Lidar application will read data published my executable node of package rplidar_hardware on topic /sensor/lidar.
Lidar application will publish detected objects on topic /detected_objects