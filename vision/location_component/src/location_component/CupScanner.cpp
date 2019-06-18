#include "location_component/CupScanner.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <ros/ros.h>

namespace location_component
{

  cv::Mat grayscaleToRGB(const cv::Mat& aGrayscale)
  {
    cv::Mat lRGB;
    cv::Mat lChannels[] = { aGrayscale, aGrayscale, aGrayscale };
    cv::merge(lChannels, 3, lRGB);
    return lRGB;
  }

  CupScanner::CupScanner(FrameCalibration& aFrameCalibration,
                         CupDetectionCalibration& aCupDetectionCalibration)
      : mFrameCalibration(aFrameCalibration),
        mCupDetectionCalibration(aCupDetectionCalibration)
  {
  }

  std::vector<DetectedCup> CupScanner::detectCups(const cv::Mat& aImage) const
  {
    cv::Mat lDisplayMat;
    return detectCups(aImage, lDisplayMat);
  }

  std::vector<DetectedCup> CupScanner::detectCups(const cv::Mat& aImage,
                                                  cv::Mat& aDisplay) const
  {
    std::vector<DetectedCup> lDetectedCups;

    cv::Mat lEdgesRaw, lEdges;
    mFrameCalibration.removeEverythingButAGV(aImage, lEdgesRaw);

    // Invert all values in the matrix.
    lEdgesRaw.forEach<uchar>(
        [](uchar& s, __attribute__((unused)) const int position[]) {
          s = (uchar)(255 - s);
        });
    cv::dilate(lEdgesRaw, lEdges,
               cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    const std::size_t lCornersSize = 4;
    cv::Point lCorners[lCornersSize];
    lCorners[0] = cv::Point();
    lCorners[1] = cv::Point(0, lEdges.rows - 1);
    lCorners[2] = cv::Point(lEdges.cols - 1, 0);
    lCorners[3] = cv::Point(lEdges.cols - 1, lEdges.rows - 1);
    for (std::size_t lIdx = 0; lIdx < lCornersSize; lIdx++)
    {
      cv::floodFill(lEdges, lCorners[lIdx], cv::Scalar(0));
    }

    cv::Mat lLeft = aImage;
    cv::Mat lRight = grayscaleToRGB(lEdges);

    std::vector<std::vector<cv::Point>> lAllContours;
    cv::findContours(lEdges, lAllContours, CV_RETR_CCOMP,
                     CV_CHAIN_APPROX_TC89_L1);

    drawContours(aDisplay, lAllContours, -1, cv::Scalar(255, 0, 0), 2);

    std::vector<std::vector<cv::Point>> lContours;
    // Remove all contours that are too small in either width or height.
    std::copy_if(lAllContours.begin(), lAllContours.end(),
                 std::back_inserter(lContours),
                 [](const std::vector<cv::Point>& lContour) mutable {
                   auto lRect = cv::boundingRect(lContour);
                   return lRect.width > ( int )cMinCupDiameter_px &&
                          lRect.height > ( int )cMinCupDiameter_px;
                 });

    for (size_t lIdx = 0; lIdx < lContours.size(); lIdx++)
    {
      DetectedCup lDetectedCup;
      lDetectedCup.mRadius = 5.0;
      cv::Moments lMu = cv::moments(lContours[lIdx]);
      cv::Point lCentroid{ ( int )(lMu.m10 / lMu.m00),
                           ( int )(lMu.m01 / lMu.m00) };
      lDetectedCup.mMidpoint = lCentroid;
      lDetectedCup.mFilled = detectCupFilled(aImage, lCentroid);
      lDetectedCups.push_back(lDetectedCup);
    }

    cv::Mat lDisplay2X;
    cv::hconcat(lLeft, lRight, lDisplay2X);
    cv::pyrDown(lDisplay2X, aDisplay);

    return lDetectedCups;
  }

  bool CupScanner::detectCupFilled(const cv::Mat& aImage,
                                   const cv::Point& aCupMidpoint) const
  {
    cv::Mat lMidpointColorBGR(1, 1, CV_8UC3);
    lMidpointColorBGR.at<cv::Vec3b>(cv::Point(0, 0)) =
        aImage.at<cv::Vec3b>(aCupMidpoint);

    cv::Mat lMidpointColorHSV(1, 1, CV_8UC3);
    cv::cvtColor(lMidpointColorBGR, lMidpointColorHSV, CV_BGR2HSV);

    ROS_DEBUG_STREAM("Cup midpoint color: "
                     << lMidpointColorHSV.at<cv::Vec3b>(cv::Point(0, 0)));
    cv::Mat lMidpointColorBW(1, 1, CV_8UC1);
    cv::inRange(
        lMidpointColorHSV, mCupDetectionCalibration.mMinFilledCupColorHSV,
        mCupDetectionCalibration.mMaxFilledCupColorHSV, lMidpointColorBW);

    return lMidpointColorBW.at<uint8_t>(cv::Point(0, 0)) != 0;
  }
} // namespace location_component
