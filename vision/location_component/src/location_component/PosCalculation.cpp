#include "location_component/PosCalculation.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace location_component
{

  PosCalculation::PosCalculation()
  {
  }

  PosCalculation::~PosCalculation()
  {
  }

  ros::Time PosCalculation::predictCupArrivalTime(
      float aCupLocationY_m,
      ros::Time aCurrentTime,
      float aAGVSpeed_m_s /*= cAGVSpeed_m_s*/) const
  {
    float lDistanceToArm_m = std::fabs(cArmY_m - aCupLocationY_m);
    float lCurrentTime_s = ( float )aCurrentTime.toSec();
    float lTimeToArm_s = lDistanceToArm_m / aAGVSpeed_m_s;
    float lPredictedArrivalTime_s = lCurrentTime_s + lTimeToArm_s;
    return ros::Time(lPredictedArrivalTime_s);
  }

  cv::Point3f PosCalculation::calculateCupLocation(cv::Point aCupScreenPos,
                                                   cv::Size aAGVFrameSize) const
  {
    cv::Point3f lCupLocation_m =
        calculatePointLocation(aCupScreenPos, aAGVFrameSize,
                               cCameraPosZ_m - cAGVDepth_m + cCupHeight_m);

    return lCupLocation_m;
  }

  cv::Point3f PosCalculation::calculateAGVLocation(cv::Point aScreenPos,
                                                   cv::Size aFrameSize) const
  {
    return calculatePointLocation(aScreenPos, aFrameSize,
                                  cCameraPosZ_m - cAGVDepth_m);
  }

  cv::Point3f
      PosCalculation::calculatePointLocation(cv::Point aScreenPos,
                                             cv::Size aFrameSize,
                                             float aObjectPositionZ_m) const
  {
    // AGV location if it was in the middle of the screen.
    cv::Point3f lObjectLocationMiddle_m =
        cv::Point3f(cCameraPosX_m, cCameraPosY_m, aObjectPositionZ_m);

    // The length of a ruler if it was stretched horizontally across the screen
    // on the Object track.
    float lObjectPlaneFrameWidth_m = (cCameraPosZ_m - aObjectPositionZ_m) *
                                     std::tan(cCameraFOV_rads / 2.0f) * 2.0f;
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

    cv::Point3f lObjectLocation_m = cv::Point3f(
        lObjectLocationMiddle_m.x + (lObjectPlaneDeviation_m.y * cCameraFlipX),
        lObjectLocationMiddle_m.y + (lObjectPlaneDeviation_m.x * cCameraFlipY),
        lObjectLocationMiddle_m.z);

    return lObjectLocation_m;
  }
} // namespace location_component
