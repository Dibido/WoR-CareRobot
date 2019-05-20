#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <opencv/opencv2.hpp>

struct Calibration
{
  std::pair<cv::Scalar, cv::Scalar> mAGVColorRange_HSV;
};

#endif /* CALIBRATION_HPP */
