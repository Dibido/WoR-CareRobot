#ifndef WEBCAMDRIVER_HPP
#define WEBCAMDRIVER_HPP

#include <boost/optional.hpp>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

namespace webcam_driver
{
  class WebcamDriver
  {
      public:
    /**
     * @brief Creates a webcam driver instance.
     *
     * @param aWebcamId The webcam ID of the webcam to connect to.
     *                  On Linux, this corresponds to the device /dev/video<id>
     */
    WebcamDriver(unsigned int aWebcamId);
    ~WebcamDriver();
    /*
     * @brief Initialises the webcam device for the maximum resolution.
     *
     * @return Whether the initialisation was successful.
     */
    bool initialise();
    /**
     * @brief Captures a frame from the webcam.
     *
     * @return The captured frame.
     */
    cv::Mat captureFrame();

      private:
    /**
     * @brief The OpenCV webcam device.
     */
    cv::VideoCapture mWebcamDevice;
  };
} // namespace webcam_driver

#endif /* WEBCAMDRIVER_HPP */
