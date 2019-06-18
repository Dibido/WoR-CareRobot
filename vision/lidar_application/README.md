After running catkin_make, lidar application can be run by 'rosrun lidar_application lidar_application_main'
After running catkin_make tests, unittests can be run by 'rosrun lidar_application lidar_application_unit_tests'

Lidar application will read data published my executable node of package rplidar_hardware on topic /sensor/lidar.
Lidar application will publish detected objects on topic /detected_objects

NOTE: It is important that on start of the application, there are no humans in the environment of the lidar. At the start of the application, the application will perform scan(s) of the environment initialscandata.

Arguments:
The application can also be given one or multiple arguments to change the behaviour of the application. An example of
calling the lidar application with argumens would be 'rosrun lidar_application lidar_application_main 0.2 3 10'. The aplication can take up to 3 arguments, the effect of the different arguments is described below:

- First argument: This describes the max difference in meters that adjacent measurement angles may differ, to still be considered as measurement angles of the same object. 0.20 meter has proven to be a good default for this. If the number is higher, it is more likely that different objects are falsely seen as one single object. If the number is lower, it is more likely that a single object is falsely seen as a few different objects.

- Second argument: Describes the minimum amount of adjacent measurements that must conflict with initial scandata to conclude there is a moving object. This can be used to filter out false-positives, a hanging cable might cause a conflict in a single measurement angle. The object is rather small though, and we might not be interested in these objects. The right number depends a bit on the environment, but 3 has been proven a decent default. Number must be 1 or higher, or an exception is thrown. If the number is 1, all slight changes are detected very early on. This makes it supersafe, but also causes alot of false-positives (hanging cable for example). If the number is higher (let say 6), the application is not likely to detect any false-positives. All small objects will be filtered out. On the other hand, it takes longer for the application to detect humans. In a 1.5 meter radius of the lidar, with 360-measurement rotations an object of 15 cm~+ width will be detected. < 15 cm will be ignored due to the fact that less then 6 measurements will find that object.

- Third argument: Amount of 360-scans performed by the lidar that are needed for initialscandata. Any following regular scans will always be compared to initialscandata. Lidar takes about 10 360-scans per second. 10 has also proven to be a good default. If the number is lower, the initialisation-time will be lower but the data will also be less detailed. If the number is higher, the initialisation-time will be longer but the initialscandata will also be more detailed.

