#ifndef POSTCALCULATION_HPP
#define POSTCALCULATION_HPP

#include "location_component/Calibration.hpp"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace location_component
{
  class PosCalculation
  {
      public:
    PosCalculation(Calibration aCalibration = Calibration());
    ~PosCalculation();

    /**
     * @brief Predicts the time when the cup arrives at the robot arm.
     */
    ros::Time predictCupArrivalTime(float aCupLocationY_m,
                                    ros::Time aCurrentTime) const;
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

    Calibration mCalibration;
  };
} // namespace location_component

#endif /* POSTCALCULATION_HPP */
