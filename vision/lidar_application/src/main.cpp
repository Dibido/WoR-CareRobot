#include "../include/ObjectDetection.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>

int main()
{
  ObjectDetection lObjectDetector;
  lObjectDetector.run();
}