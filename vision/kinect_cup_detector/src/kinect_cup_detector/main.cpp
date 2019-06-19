#include "kinect_cup_detector/CupDetector.hpp"

int main(int argc, char** argv)
{
  // Setup ROS
  ros::init(argc, argv, "kinect_cup_detector");
  // Create CupDetector
  if (argc == 2 && std::string(argv[1]) == std::string("-d"))
  {
    std::cout << "Started in debug mode." << std::endl;
    CupDetector lCupDetector(true);
    // Handle the incoming messages
    ros::spin();
  }
  else
  {
    CupDetector lCupDetector(false);
    // Handle the incoming messages
    ros::spin();
  }
  return 0;
}