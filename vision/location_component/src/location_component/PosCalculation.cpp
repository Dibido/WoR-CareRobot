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
      ros::Time aCurrentTime) const
  {
    float lDistanceToArm_m = std::fabs(cArmY_m - aCupLocationY_m);
    float lCurrentTime_s = ( float )aCurrentTime.toSec();
    float lTimeToArm_s = lDistanceToArm_m / mAGVSpeed_m_s;
    float lPredictedArrivalTime_s = lCurrentTime_s + lTimeToArm_s;
    return ros::Time(lPredictedArrivalTime_s);
  }

  cv::Point3f PosCalculation::calculateCupLocation(cv::Point aAGVScreenPos,
                                                   cv::Size aAGVFrameSize,
                                                   __attribute__((unused))
                                                   cv::Point aCupScreenPos,
                                                   __attribute__((unused))
                                                   cv::Size aCupFrameSize) const
  {
    cv::Point3f lAGVLocation_m =
        calculateAGVLocation(aAGVScreenPos, aAGVFrameSize);
    /* cv::Point2f lRelativeCupLocation = */
    /*     calculateRelativeCupLocation(aCupScreenPos, aCupFrameSize); */

    /*     cv::Point2f lCupDeviationMiddle_m = */
    /*         cv::Point2f((lRelativeCupLocation.x - 0.5f) * cAGVWidth_m, */
    /*                     (lRelativeCupLocation.y - 0.5f) * cAGVHeight_m) */
    /*         * 1.0f; */

    /*     cv::Point3f lCupLocation_m = */
    /*         cv::Point3f(lAGVLocation_m.x + (lCupDeviationMiddle_m.y *
     * cCameraFlipX), */
    /*                     lAGVLocation_m.y + (lCupDeviationMiddle_m.x *
     * cCameraFlipY), */
    /*                     lAGVLocation_m.z + cCupHeight_m); */

    // For the moment, assume the cup is at the middle of the AGV.
    cv::Point3f lCupLocation_m = cv::Point3f(lAGVLocation_m.x, lAGVLocation_m.y,
                                             lAGVLocation_m.z + cCupHeight_m);

    return lCupLocation_m;
  }

  cv::Point2f
      PosCalculation::calculateRelativeCupLocation(cv::Point aScreenPos,
                                                   cv::Size aFrameSize) const
  {
    float lXPosFactor =
        static_cast<float>(aScreenPos.x) / static_cast<float>(aFrameSize.width);
    float lYPosFactor = static_cast<float>(aScreenPos.y) /
                        static_cast<float>(aFrameSize.height);

    return cv::Point2f(lXPosFactor, lYPosFactor);
  }

  cv::Point3f PosCalculation::calculateAGVLocation(cv::Point aScreenPos,
                                                   cv::Size aFrameSize) const
  {
    // AGV location if it was in the middle of the screen.
    cv::Point3f lAGVLocationMiddle_m =
        cv::Point3f(cCameraPosX_m, cCameraPosY_m, cCameraPosZ_m - cAGVDepth_m);

    // The length of a ruler if it was stretched horizontally across the screen
    // on the AGV track.
    float lAGVPlaneFrameWidth_m =
        cAGVDepth_m * std::tan(cCameraFOV_rads / 2.0f) * 2.0f;
    // The size of the plane created by two rulers for the whole screen.
    cv::Point2f lAGVPlaneFrameSize_m = cv::Point2f(
        lAGVPlaneFrameWidth_m, lAGVPlaneFrameWidth_m *
                                   static_cast<float>(aFrameSize.height) /
                                   static_cast<float>(aFrameSize.width));

    // The factor of the AGV position on the frame [0.0, 1.0]
    float lXPosFactor =
        static_cast<float>(aScreenPos.x) / static_cast<float>(aFrameSize.width);
    float lYPosFactor = static_cast<float>(aScreenPos.y) /
                        static_cast<float>(aFrameSize.height);

    // The 2D AGV deviation from the middle of the screen calculated to the
    // world size.
    cv::Point2f lAGVPlaneDeviation_m =
        cv::Point2f(lAGVPlaneFrameSize_m.x * (lXPosFactor - 0.5f),
                    lAGVPlaneFrameSize_m.y * (lYPosFactor - 0.5f));

    cv::Point3f lAGVLocation_m = cv::Point3f(
        lAGVLocationMiddle_m.x + (lAGVPlaneDeviation_m.y * cCameraFlipX),
        lAGVLocationMiddle_m.y + (lAGVPlaneDeviation_m.x * cCameraFlipY),
        lAGVLocationMiddle_m.z);

    return lAGVLocation_m;
  }

  float PosCalculation::getAGVSpeed_m_s()
  {
    return mAGVSpeed_m_s;
  }

  void PosCalculation::setAGVSpeed_m_s(const float aAGVSpeed_m_s)
  {
    if(aAGVSpeed_m_s < 0)
    {
      throw std::range_error("AGV speed cannot be lower than zero");
    }
    
    mAGVSpeed_m_s = aAGVSpeed_m_s;
  }

} // namespace location_component
