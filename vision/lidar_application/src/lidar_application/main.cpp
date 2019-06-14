#include "lidar_application/ObjectDetection.hpp"
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ObjectDetection");

  // 0.20 meter had been proven a good default.
  double lMaxDifference_m = 0.20;

  uint16_t lMinNumberArgv = 2;

  if (argc == 3)
  {
    ROS_INFO("Setting lMaxDifference_m to %s", argv[1]);
    lMaxDifference_m = strtod(argv[1], NULL);

    ROS_INFO("Setting lMinNumberArgv to %s", argv[2]);
    lMinNumberArgv = std::stoi(argv[2]);
  }

  lidar_application::ObjectDetection lObjectDetector(lMaxDifference_m);
  lObjectDetector.cObjectMinNumberOfAdjacentMeasurementsDebug = lMinNumberArgv;
  lObjectDetector.run();
}
