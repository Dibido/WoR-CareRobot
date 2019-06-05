#include "location_component/FrameCalibration.hpp"

namespace location_component
{
  /**
   * @brief Construct a new Frame Calibration:: Frame Calibration object
   *
   * @param aAGVFrameCalibration - Struct with all the calibration info
   */
  FrameCalibration::FrameCalibration(AGVFrameCalibration aAGVFrameCalibration)
      : mAGVFrameCalibration(aAGVFrameCalibration)
  {
  }

  /**
   * @brief This function will edit the frame and remove everything but the AGV
   *
   * @param aSource - The source matrix used for transformation
   * @param aDestination - The matrix that will be transformed
   */
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