#ifndef FRAME_CALIBRATION_HPP
#define FRAME_CALIBRATION_HPP

#include "location_component/AGVFrameCalibration.hpp"
#include "location_component/FrameCalibration.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace location_component
{
  class FrameCalibration
  {
      public:
    /**
     * @brief Construct a new Frame Calibration object
     *
     */
    FrameCalibration(AGVFrameCalibration aAGVFrameCalibration);
    /**
     * @brief Destroy the Frame Calibration object
     *
     */
    ~FrameCalibration() = default;

    void removeEverythingButAGV(const cv::Mat& aSource,
                                cv::Mat& aDestination) const;

      private:
    AGVFrameCalibration mAGVFrameCalibration;
  };

} // namespace location_component

#endif