#ifndef DETECTED_FRAME_HPP
#define DETECTED_FRAME_HPP

#include "location_component/DetectedAGV.hpp"
#include "location_component/DetectAGV.hpp"

namespace location_component
{
  struct DetectedFrame
  {
    DetectedAGV mDetectedAGV;
    std::vector<DetectedCup> mDetectedCups;
    cv::Size mAGVFrameSize;
    cv::Size mCupFrameSize;
  };

} // namespace location_component
#endif