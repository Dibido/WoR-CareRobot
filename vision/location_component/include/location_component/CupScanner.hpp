#ifndef CUPSCANNER_HPP
#define CUPSCANNER_HPP

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "location_component/FrameCalibration.hpp"

namespace location_component
{
  /**
   * @brief The minimum diameter of the detected cups in pixels.
   */
  const unsigned int cMinCupDiameter_px = 100;

  struct DetectedCup
  {
    double mRadius;
    bool mFilled;
    cv::Point mMidpoint;
  };

  class CupScanner
  {
      public:
    CupScanner(FrameCalibration& aFrameCalibration);
    ~CupScanner() = default;

    /**
     * @brief This function will use a frame and detect the cups on the frame.
     * It will register all the cups and the different states of the cups.
     *
     * @param aImage - The image on which to register the different cups.
     * @param aDisplay - A debug frame with all the information of the detected
     * cups.
     * @return std::vector<DetectedCup> - A collection of the detected cups.
     */
    std::vector<DetectedCup> detectCups(const cv::Mat& aImage,
                                        cv::Mat& aDisplay) const;
    /**
     * @brief Overload for CupScanner::detectCups without the output display
     * matrix.
     */
    std::vector<DetectedCup> detectCups(const cv::Mat& aImage) const;

    private:
    FrameCalibration& mFrameCalibration;
  };
} // namespace location_component

#endif /* CUPSCANNER_HPP */
