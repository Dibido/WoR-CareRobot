#include "webcam_driver/WebcamDriver.hpp"
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

  webcam_driver::WebcamDriver lWebcamDriver(lWebcamId);
  if (!lWebcamDriver.initialise())
  {
    ROS_WARN_STREAM("Failed to initialise webcam with id " << lWebcamId);
    return 2;
  }

  image_transport::ImageTransport lImageTransport(lNodeHandle);
  image_transport::Publisher lPublisher =
      lImageTransport.advertise(webcam_driver_constants::cWebcamTopic, 1);

  if (lDebug)
  {
    cv::namedWindow("a", cv::WINDOW_AUTOSIZE);
  }
  cv::Mat lFrame;
  while (ros::ok())
  {
    cv::Mat lCapturedFrame = lWebcamDriver.captureFrame();
    sensor_msgs::ImagePtr lMsg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", lCapturedFrame)
            .toImageMsg();
    lPublisher.publish(lMsg);
    ros::spinOnce();

    if (lDebug)
    {
      cv::imshow("a", lWebcamDriver.captureFrame());
    }

    int lChar = cv::waitKey(1000 / webcam_driver_constants::cRefreshRate);
    if (lChar == 27)
    {
      break;
    }
  }

  return 0;
}
