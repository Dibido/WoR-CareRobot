#ifndef WEBCAMDRIVER_HPP
#define WEBCAMDRIVER_HPP

#include <boost/optional.hpp>
#include <iostream>
#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace webcam_driver
{
  class WebcamDriver
  {
      public:
    WebcamDriver();
    ~WebcamDriver();
  };
} // namespace webcam_driver

#endif /* WEBCAMDRIVER_HPP */
