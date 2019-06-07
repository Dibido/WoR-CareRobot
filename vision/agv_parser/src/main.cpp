#include "agv_parser/AgvParser.hpp"

#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "AgvParser");
  if (argc == 1)
  {
    // Create with default port
    agv_parser::AgvParser lAgvParser("/dev/ttyUSB0");
    lAgvParser.run();
  }
  else if (argc == 2)
  {
    // Create with port from arguments
    agv_parser::AgvParser lAgvParser(argv[1]);
    lAgvParser.run();
  }
  return 0;
}