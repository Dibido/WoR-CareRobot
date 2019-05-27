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

    Calibration mCalibration;
  };
} // namespace location_component

#endif /* POSTCALCULATION_HPP */
