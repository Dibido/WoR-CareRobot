#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <opencv/opencv2.hpp>

namespace location_component
{
  struct Calibration
  {
    std::pair<cv::Scalar, cv::Scalar> mAGVColorRange_HSV;
  };
} // namespace location_component

#endif /* CALIBRATION_HPP */
