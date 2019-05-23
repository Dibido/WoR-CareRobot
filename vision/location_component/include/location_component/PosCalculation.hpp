#ifndef POSTCALCULATION_HPP
#define POSTCALCULATION_HPP

#include <opencv2/opencv.hpp>

namespace location_component
{
  // Camera position
  const float cCameraPosX_m = 0.35f;
  const float cCameraPosY_m = -4.0f;
  const float cCameraPosZ_m = 1.8f;
  const float cCameraFOV_rads = 1.047f;

  const float cCameraFlipX = -1.0f;
  const float cCameraFlipY = -1.0f;

  const float cCupHeight_m = 0.099f;
  const float cAGVDepth_m = 0.680f;

  const float cAGVWidth_m = 0.350f;
  const float cAGVHeight_m = 0.400f;

  class PosCalculation
  {
      public:
    PosCalculation();
    ~PosCalculation();
    cv::Point3f calculateCupLocation(cv::Point aAGVScreenPos,
                                     cv::Size aAGVFrameSize,
                                     cv::Point aCupScreenPos,
                                     cv::Size aCupFrameSize);
    cv::Point3f calculateAGVLocation(cv::Point aScreenPos,
                                     cv::Size aAGVFrameSize);

      private:
    cv::Point2f calculateRelativeCupLocation(cv::Point aScreenPos,
                                             cv::Size aFrameSize);
  };
} // namespace location_component

#endif /* POSTCALCULATION_HPP */
