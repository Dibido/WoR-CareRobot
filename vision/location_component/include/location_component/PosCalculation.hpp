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
     * @param aCupScreenPos The position of the cup on the screen.
     * @param aAGVFrameSize The size of the screen frame.
     */
    cv::Point3f calculateCupLocation(cv::Point aCupScreenPos,
                                     cv::Size aAGVFrameSize) const;
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
     * @brief Calculates a point location in the world based on screen
     * coordinates.
     *
     * @param aScreenPos The position of the point on the screen.
     * @param aFrameSize The size of the screen frame.
     * @param fObjectPositionZ_m The Z-position of the object in the world.
     */
    cv::Point3f calculatePointLocation(cv::Point aScreenPos,
                                       cv::Size aFrameSize,
                                       float fObjectPositionZ_m) const;
  };
} // namespace location_component

#endif /* POSTCALCULATION_HPP */
