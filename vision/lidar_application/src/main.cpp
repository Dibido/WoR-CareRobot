#include "../include/ObjectDetection.hpp"

#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ObjectDetection");

  ObjectDetection lObjectDetector;
  lObjectDetector.run();
}