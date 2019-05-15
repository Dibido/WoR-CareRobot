# sim_sonar

This package publishes arduino sensor data through a specific serial to a ROS topic.



## Install

### Ultrasonic library

This library reads data from the ultrasonic sensor.

1. Download the [arduino ultrasonic library] from github.
1. Open the arduino IDE.
1. Click on `sketch` in the menu item at the top.
1. Click on `include library` .
1. Click on `add .zip library`.
1. Select the libraries zip file you just downloaded from git.
1. Click `ok`.

### Rosserial Library

This library makes it possible to publish ROS messages over the serial

1. Install and deploy the [arduino ros library] by executing commands in the command block below
```
$ sudo apt install ros-melodic-rosserial
$ sudo apt install ros-melodic-rosserial-arduino
```

2. Deploy the [arduino ros library].
    *  In the Arduino IDE click on the `file` > `preferences` menu to find your sketchbook location.

```
$ cd <sketchbook>/libraries
$ rm -rf ros_lib
$ rosrun rosserial_arduino make_libraries.py .
```

### Ultrasonic publisher library

This library combines the ultrasonic library and the rosserial library.

1. Copy the folders inside `<YOUR_PATH_TO_sen_sonar>/arduino/libraries` to the `<sketchbook>` folder.
    *  In the Arduino IDE click on the `file` > `preferences` menu to find your sketchbook location.

[arduino ultrasonic library]: https://github.com/ErickSimoes/Ultrasonic
[arduino ros library]: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup



## Build

1. Open the `<YOUR_PATH_TO_sen_sonar>/arduino/sketch/sketch.ino` arduino sketch to the arduino.
1. Connect the sensors according to the wire connections specified in the top of the sketch file.
1. Connect the arduino with USB and select the right `port` and `board` in the `tools` menu
1. Upload the arduino sketch.
1. `$ source devel/setup.bash`.
1. `$ cd <catkin_workspace>`. . .

## Run

1. `$ roslaunch sen_sonar launch.launch serial:=<YOUR_SERIAL_HERE>`.
    * The serial can deffer. You can find the right serial in the `tools` > `port` menu at the top.
1. `$ rostopic echo /sensor/sonar`.
