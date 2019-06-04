#include "webcam_driver/WebcamDriver.hpp"
#include "ros/ros.h"

namespace webcam_driver
{
  WebcamDriver::WebcamDriver(unsigned int aWebcamId) : mWebcamDevice(aWebcamId)
  {
  }

  WebcamDriver::~WebcamDriver()
  {
  }

  bool WebcamDriver::initialise()
  {
    if (!mWebcamDevice.isOpened())
    {
      return false;
    }
    // Set the webcam resolution to a huge size.
    // OpenCV will automatically set it to the highest resolution supported by
    // the camera.
    const unsigned int cHugeSize = 10000;
    mWebcamDevice.set(CV_CAP_PROP_FRAME_WIDTH, 10000);
    mWebcamDevice.set(CV_CAP_PROP_FRAME_HEIGHT, 10000);
    ROS_DEBUG_STREAM("Got webcam device with resolution "
                     << mWebcamDevice.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
                     << mWebcamDevice.get(CV_CAP_PROP_FRAME_HEIGHT));

    return mWebcamDevice.isOpened();
  }

  cv::Mat WebcamDriver::captureFrame()
  {
    cv::Mat lFrame;
    mWebcamDevice.read(lFrame);
    return lFrame;
  }
} // namespace webcam_driver
