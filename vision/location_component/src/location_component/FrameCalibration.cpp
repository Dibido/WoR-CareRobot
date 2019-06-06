#include "location_component/FrameCalibration.hpp"

namespace location_component
{

  FrameCalibration::FrameCalibration(AGVFrameCalibration aAGVFrameCalibration)
      : mAGVFrameCalibration(aAGVFrameCalibration)
  {
  }

  void FrameCalibration::removeEverythingButAGV(const cv::Mat& aSource,
                                                cv::Mat& aDestination) const
  {
    cv::inRange(
        aSource,
        cv::Scalar(mAGVFrameCalibration.cHLow, mAGVFrameCalibration.cSLow,
                   mAGVFrameCalibration.cVLow),
        cv::Scalar(mAGVFrameCalibration.cHHigh, mAGVFrameCalibration.cSHigh,
                   mAGVFrameCalibration.cVHigh),
        aDestination);
  }

} // namespace location_component