#include "kinect_cup_detector/CupDetector.hpp"

int main(int argc, char** argv)
{
  // Setup ROS
  ros::init(argc, argv, "kinect_cup_detector");
  // Create CupDetector
  if (argc == 2 && argv[1] == "-d")
  {
    CupDetector lCupDetector(true);
    // Handle the incoming messages
    while (ros::ok())
    {
      ros::spinOnce();
    }
  }
  else
  {
    CupDetector lCupDetector(false);
    // Handle the incoming messages
    while (ros::ok())
    {
      ros::spinOnce();
    }
  }
  return 0;
}