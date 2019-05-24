#include "lidar_application/ObjectDetection.hpp"
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ObjectDetection");

  // 0.20 meter has proved to be a good default.
  double lMaxDifference_m = 0.20;

  if (argc == 2)
  {
    std::cout << "Setting lMaxDifference_m to " << argv[1] << std::endl;
    lMaxDifference_m = strtod(argv[1], NULL);
  }

  lidar_application::ObjectDetection lObjectDetector(lMaxDifference_m);
  lObjectDetector.run();
}