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
    cv::VideoCapture mWebcamDevice;
  };
} // namespace webcam_driver

#endif /* WEBCAMDRIVER_HPP */
