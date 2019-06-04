#include "location_component/PosCalculation.hpp"
#include "location_component/Calibration.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace location_component
{

  PosCalculation::PosCalculation(CupDetectionCalibration aCalibration /* = Calibration()*/)
      : mCalibration(aCalibration)
  {
  }

  PosCalculation::~PosCalculation()
  {
  }

  ros::Time PosCalculation::predictCupArrivalTime(float aCupLocationY_m,
                                                  ros::Time aCurrentTime) const
  {
    float lDistanceToArm_m = std::fabs(mCalibration.mArmY_m - aCupLocationY_m);
    float lCurrentTime_s = ( float )aCurrentTime.toSec();
    float lTimeToArm_s = lDistanceToArm_m / mAGVSpeed_m_s;
    float lPredictedArrivalTime_s = lCurrentTime_s + lTimeToArm_s;
    return ros::Time(lPredictedArrivalTime_s);
  }

  cv::Point3f PosCalculation::calculateCupLocation(cv::Point aCupScreenPos,
                                                   cv::Size aAGVFrameSize) const
  {
    cv::Point3f lCupLocation_m = calculatePointLocation(
        aCupScreenPos, aAGVFrameSize,
        mCalibration.mCameraPosZ_m - mCalibration.mAGVDepth_m +
            mCalibration.mCupHeight_m);

    return lCupLocation_m;
  }

  cv::Point3f PosCalculation::calculateAGVLocation(cv::Point aScreenPos,
                                                   cv::Size aFrameSize) const
  {
    return calculatePointLocation(aScreenPos, aFrameSize,
                                  mCalibration.mCameraPosZ_m -
                                      mCalibration.mAGVDepth_m);
  }

  cv::Point3f
      PosCalculation::calculatePointLocation(cv::Point aScreenPos,
                                             cv::Size aFrameSize,
                                             float aObjectPositionZ_m) const
  {
    // Point location if it was in the middle of the screen.
    cv::Point3f lObjectLocationMiddle_m =
        cv::Point3f(mCalibration.mCameraPosX_m, mCalibration.mCameraPosY_m,
                    aObjectPositionZ_m);

    // The length of a ruler if it was stretched horizontally across the screen
    // on the Object track.
    float lObjectPlaneFrameWidth_m =
        (mCalibration.mCameraPosZ_m - aObjectPositionZ_m) *
        std::tan(mCalibration.mCameraFOV_rads / 2.0f) * 2.0f;
    // The size of the plane created by two rulers for the whole screen.
    cv::Point2f lObjectPlaneFrameSize_m = cv::Point2f(
        lObjectPlaneFrameWidth_m, lObjectPlaneFrameWidth_m *
                                      static_cast<float>(aFrameSize.height) /
                                      static_cast<float>(aFrameSize.width));

    // The factor of the Object position on the frame [0.0, 1.0]
    float lXPosFactor =
        static_cast<float>(aScreenPos.x) / static_cast<float>(aFrameSize.width);
    float lYPosFactor = static_cast<float>(aScreenPos.y) /
                        static_cast<float>(aFrameSize.height);

    // The 2D Object deviation from the middle of the screen calculated to the
    // world size.
    cv::Point2f lObjectPlaneDeviation_m =
        cv::Point2f(lObjectPlaneFrameSize_m.x * (lXPosFactor - 0.5f),
                    lObjectPlaneFrameSize_m.y * (lYPosFactor - 0.5f));

    cv::Point3f lObjectLocation_m =
        cv::Point3f(lObjectLocationMiddle_m.x +
                        (lObjectPlaneDeviation_m.y * mCalibration.mCameraFlipX),
                    lObjectLocationMiddle_m.y +
                        (lObjectPlaneDeviation_m.x * mCalibration.mCameraFlipY),
                    lObjectLocationMiddle_m.z);

    return lObjectLocation_m;
  }

  float PosCalculation::getAGVSpeed_m_s() const
  {
    return mAGVSpeed_m_s;
  }

  void PosCalculation::setAGVSpeed_m_s(const float aAGVSpeed_m_s)
  {
    if (aAGVSpeed_m_s < 0.0)
    {
      throw std::range_error("AGV speed cannot be lower than zero");
    }

    mAGVSpeed_m_s = aAGVSpeed_m_s;
  }

} // namespace location_component
