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
      cv::imshow("WebcamDriver debug", lCapturedFrame);
    }
  }

} // namespace webcam_driver
