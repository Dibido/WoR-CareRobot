#ifndef DETECTED_AGV_HPP
#define DETECTED_AGV_HPP

#include "location_component/DetectAGV.hpp"
#include <vector>

namespace location_component
{
  struct DetectedAGV
  {
    std::vector<cv::Point> mCorners;
    cv::Point mMidpoint;
    cv::Rect mBoundRect;
    cv::Mat mAGVFrame;
  };
} // namespace location_component
#endif