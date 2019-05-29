# README

Packages can be compiled by executing the following command from the workspace root:

```
catkin_make
```

Unittests for **<u>all</u>** packages can be compiled by executing the following command from the workspace root:

```
catkin_make tests
```

Unittests per package can be run by the following command:
```
rosrun ${PACKAGE_NAME} ${PACKAGE_NAME}_unit_tests
```


Code coverage per package can be generated by executing the following command from the workspace root:

```
catkin_make -DCODE_COVERAGE=1 ${PACKAGE_NAME}_coverage
```

Code coverage results can be found in the following folder:
```
${ROS_WORKSPACE}/build/${PACKAGE_NAME}/${PACKAGE_NAME}_coverage
```

## Simulation guide

1. First install the required libraries:
```
sudo apt install libsdl2-dev
```
Not found error :
```
sudo apt install libSDL2-dev
```
Unmet dependency error:
```
sudo apt-get install aptitude
sudo aptitude install libsdl2-dev
```
2. Clone kinect repo:
```
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
sudo apt-get install build-essential cmake pkg-config
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libturbojpeg0-dev
sudo apt-get install libglfw3-dev
sudo apt-get install libopenni2-dev
```
3. Execute in libfreenect2 root directory:
```
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
```

4. Franka Panda requires the following library to work:

```
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
cd ~
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
catkin_make -DFranka_DIR:PATH=~/libfranka/build
```
5. edit the Cmakelists:
```
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.3.0 REQUIRED)
 
include_directories(
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
)
 
catkin_package(
  LIBRARIES franka_state_controller franka_control_services
)
```
5.1 If you want to add an executable:
```
target_link_libraries( %executableName% ${catkin_LIBRARIES} ${Franka_LIBRARIES})
```
6. execute:
```
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
```
7. Navigate to git repo and execute:
```
git submodule init
git submodule update
```