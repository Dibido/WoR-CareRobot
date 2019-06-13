#include "webcam_driver/WebcamPublisher.hpp"
#include <cv_bridge/cv_bridge.h>
#include <exception>

namespace webcam_driver
{

  WebcamPublisher::WebcamPublisher(unsigned int aWebcamId,
                                   bool aDebug,
                                   const std::string& aTopicName,
                                   ros::NodeHandle& aNH)
      : mWebcamDriver(aWebcamId), mImageTransport(aNH), mDebug(aDebug)
  {
    mPublisher = mImageTransport.advertise(aTopicName, 1);

    if (!mWebcamDriver.initialise())
    {
      throw std::runtime_error("Failed to initialise webcam with id " +
                               std::to_string(aWebcamId));
    }

    if (mDebug)
    {
      cv::namedWindow("WebcamDriver debug", cv::WINDOW_AUTOSIZE);
    }
  }

  WebcamPublisher::~WebcamPublisher()
  {
  }

  void WebcamPublisher::update()
  {
    cv::Mat lCapturedFrame = mWebcamDriver.captureFrame();
    sensor_msgs::ImagePtr lMsg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", lCapturedFrame)
            .toImageMsg();
    mPublisher.publish(lMsg);
    if (mDebug)
    {
      circle(lCapturedFrame,
             cvPoint(lCapturedFrame.size().width / 2,
                     lCapturedFrame.size().height / 2),
             3, cv::Scalar(255, 255, 0), 2);

      cv::imshow("WebcamDriver debug", lCapturedFrame);
    }
  }

  void WebcamPublisher::mainLoop(unsigned int aRefreshRate)
  {
    while (ros::ok())
    {
      update();
      ros::spinOnce();

      // Wait for (1000 / refresh rate) milliseconds.
      int lChar = cv::waitKey(1000 / aRefreshRate);
      const int cKeycodeEscape = 27;
      if (lChar == cKeycodeEscape)
      {
        break;
      }
    }
  }

} // namespace webcam_driver
