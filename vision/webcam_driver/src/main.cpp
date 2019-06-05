#include "webcam_driver/WebcamDriver.hpp"
#include "webcam_driver/WebcamPublisher.hpp"
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <vector>

namespace webcam_driver_constants
{
  const std::string cWebcamTopic = "/sensor/webcam";
  const std::string cComponentName = "webcam_driver";
  const unsigned int cRefreshRate = 30;
} // namespace webcam_driver_constants

int main(int argc, char** argv)
{
  ros::init(argc, argv, webcam_driver_constants::cComponentName);
  ros::NodeHandle lNodeHandle;

  unsigned int lWebcamId = 0;
  if (argc > 1 && std::string(argv[1]) != "-h" && argv[1][0] == '-')
  {
    lWebcamId = static_cast<unsigned int>(atoi(argv[1] + 1));
  }
  else
  {
    ROS_INFO_STREAM("Usage: webcam_driver -<webcam device id> [-d]");
    return 1;
  }

  bool lDebug = false;
  if (argc > 2 && std::string(argv[2]) == "-d")
  {
    lDebug = true;
  }

  webcam_driver::WebcamPublisher lWebcamPublisher(
      lWebcamId, lDebug, webcam_driver_constants::cWebcamTopic, lNodeHandle);

  lWebcamPublisher.mainLoop(webcam_driver_constants::cRefreshRate);

  return 0;
}
