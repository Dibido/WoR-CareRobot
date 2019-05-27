#ifndef POSTCALCULATION_HPP
#define POSTCALCULATION_HPP

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace location_component
{
  // Camera position
  const float cCameraPosX_m = 0.35f;
  const float cCameraPosY_m = -4.0f;
  const float cCameraPosZ_m = 0.8f;

  // Camera FOV
  const float cCameraFOV_rads = 1.047f;

  // The position where the arm will pick up the cup
  const float cArmY_m = 0.0f;

  // Compensate for flipped camera
  const float cCameraFlipX = -1.0f;
  const float cCameraFlipY = -1.0f;

  // Cup and AGV size
  const float cCupHeight_m = 0.099f;
  const float cCupDiameter_m = 0.071f;
  const float cAGVDepth_m = 0.680f;
  const float cAGVWidth_m = 0.350f;
  const float cAGVHeight_m = 0.400f;

  // AGV speed
  const float cAGVSpeed_m_s = 0.220f;

  class PosCalculation
  {
      public:
    PosCalculation();
    ~PosCalculation();

    /**
     * @brief Predicts the time when the cup arrives at the robot arm.
     */
    ros::Time predictCupArrivalTime(float aCupLocationY_m,
                                    ros::Time aCurrentTime,
                                    float aAGVSpeed_m_s = cAGVSpeed_m_s) const;
    /**
     * @brief Calculates the cup location.
     *
     * @param aAGVScreenPos The position of the AGV on the screen.
     * @param aAGVFrameSize The size of the AGV screen frame.
     * @param aCupScreenPos The size of the cup on the cup screen.
     * @param aCupFrameSize The size of the cup screen frame.
     */
    cv::Point3f calculateCupLocation(cv::Point aAGVScreenPos,
                                     cv::Size aAGVFrameSize,
                                     cv::Point aCupScreenPos,
                                     cv::Size aCupFrameSize) const;
    /**
     * @brief Calculates the AGV location in the world based on screen
     * coordinates.
     *
     * @param aScreenPos The position of the AGV on the screen.
     * @param aFrameSize The size of the screen frame.
     */
    cv::Point3f calculateAGVLocation(cv::Point aScreenPos,
                                     cv::Size aAGVFrameSize) const;

      private:
    /**
     * @brief Calculates the relative location of the cup on the AGV [0.0-1.0].
     *
     * @param aScreenPos The size of the cup on the cup screen.
     * @param aFrameSize The size of the cup screen frame.
     */
    cv::Point2f calculateRelativeCupLocation(cv::Point aScreenPos,
                                             cv::Size aFrameSize) const;
  };
} // namespace location_component

#endif /* POSTCALCULATION_HPP */
