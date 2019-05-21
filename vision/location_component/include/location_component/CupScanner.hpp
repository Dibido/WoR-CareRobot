#ifndef ARUCOSCANNER_HPP
#define ARUCOSCANNER_HPP

#include <opencv2/opencv.hpp>

namespace location_component
{
  struct DetectedCup
  {
    double mRadius;
    bool mFilled;
    cv::Point mMidpoint;
  };

  class CupScanner
  {
      public:
    CupScanner();
    ~CupScanner();
    std::vector<DetectedCup> scan(const cv::Mat& image, cv::Mat& display);
  };
} // namespace location_component

#endif /* ARUCOSCANNER_HPP */
