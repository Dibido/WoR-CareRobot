#include "kinect_cup_detector/CupDetector.hpp"

int main(int argc, char** argv)
{
  // Setup ROS
  ros::init(argc, argv, "kinect_cup_detector");
  // Create CupDetector
  CupDetector lCupDetector{};
  // Handle the incoming messages
  while (ros::ok())
  {
    ros::spinOnce();
  }
}