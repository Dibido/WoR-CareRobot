#include "lidar_application/ObjectDetection.hpp"
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ObjectDetection");

  // 0.20 meter had been proven a good default, see ObjectDetection.hpp
  // mMaxDistanceDifference
  double lMaxDifference_m = 0.20;

  // 3 has been proven a good default, see ObjectDetection.hpp
  unsigned int lMinNumberAdjacentAngles = 3;

  // 10 has proven to be a good default, 10 cycles of the lidar takes about a
  // second see ObjectDetection.hpp
  unsigned int lNumberOfInitialScanRounds = 10;

  if (argc == 2)
  {
    ROS_INFO("Setting lMaxDifference_m to %s", argv[1]);
    lMaxDifference_m = strtod(argv[1], NULL);
  }
  else if (argc == 3)
  {
    ROS_INFO("Setting lMaxDifference_m to %s", argv[1]);
    lMaxDifference_m = strtod(argv[1], NULL);

    ROS_INFO("Setting lMinNumberAdjacentAngles to %s", argv[2]);
    lMinNumberAdjacentAngles = std::stoi(argv[2]);
  }
  else if (argc == 4)
  {
    ROS_INFO("Setting lMaxDifference_m to %s", argv[1]);
    lMaxDifference_m = strtod(argv[1], NULL);

    ROS_INFO("Setting lMinNumberAdjacentAngles to %s", argv[2]);
    lMinNumberAdjacentAngles = std::stoi(argv[2]);

    ROS_INFO("Setting lNumberOfInitialScanRounds to %s", argv[3]);
    lNumberOfInitialScanRounds = std::stoi(argv[3]);
  }

  lidar_application::ObjectDetection lObjectDetector(
      lMaxDifference_m, true, lMinNumberAdjacentAngles,
      lNumberOfInitialScanRounds);
  lObjectDetector.run();
}
