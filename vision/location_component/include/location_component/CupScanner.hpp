#ifndef ARUCOSCANNER_HPP
#define ARUCOSCANNER_HPP

#include <opencv2/opencv.hpp>

namespace location_component
{
  /**
   * @brief Information of the detected cups in a object.
   *
   */
  struct DetectedCup
  {
    double mRadius;
    bool mFilled;
    cv::Point mMidpoint;
  };

  class CupScanner
  {
      public:
    CupScanner();
    ~CupScanner();

    /**
     * @brief This function will use a frame and detect the cups on the frame.
     * It will register all the cups and the different states of the cups.
     *
     * @param image - The image u want to register the different cups.
     * @param display - A debug frame with all the information of the detected
     * cups.
     * @return std::vector<DetectedCup> - A vector with a alement for all the
     * detected cups.
     */
    std::vector<DetectedCup> scan(const cv::Mat& image, cv::Mat& display);
  };
} // namespace location_component

#endif /* ARUCOSCANNER_HPP */
