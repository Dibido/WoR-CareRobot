#include "location_component/FrameCalibration.hpp"
#include "opencv2/opencv.hpp"

namespace location_component
{

  FrameCalibration::FrameCalibration(AGVFrameCalibration aAGVFrameCalibration)
      : mAGVFrameCalibration(aAGVFrameCalibration)
  {
  }

  void FrameCalibration::removeEverythingButAGV(const cv::Mat& aSource,
                                                cv::Mat& aDestination) const
  {
    cv::Mat lSourceHSV;
    cv::cvtColor(aSource, lSourceHSV, CV_BGR2HSV);

    cv::Scalar lScalarLow(mAGVFrameCalibration.cHLow,
                          mAGVFrameCalibration.cSLow,
                          mAGVFrameCalibration.cVLow);

    cv::Scalar lScalarHigh(mAGVFrameCalibration.cHHigh,
                           mAGVFrameCalibration.cSHigh,
                           mAGVFrameCalibration.cVHigh);

    cv::inRange(lSourceHSV, lScalarLow, lScalarHigh, aDestination);

    if (mAGVFrameCalibration.mDebugStatus)
    {
      cv::Mat lLeftImg, lRightImg, lDebugImg, lDestinationRGB;
      cv::Mat lDestinationChannels[] = { aDestination, aDestination,
                                         aDestination };
      cv::merge(lDestinationChannels, 3, lDestinationRGB);
      cv::pyrDown(aSource, lLeftImg);
      cv::pyrDown(lDestinationRGB, lRightImg);
      cv::hconcat(lLeftImg, lRightImg, lDebugImg);

      cv::imshow("FrameCalibration debug", lDebugImg);
    }
  }

} // namespace location_component
