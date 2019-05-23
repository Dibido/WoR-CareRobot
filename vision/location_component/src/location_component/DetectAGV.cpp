#include "location_component/DetectAGV.hpp"
#include "location_component/CupScanner.hpp"
#include "location_component/PosCalculation.hpp"
#include <ros/ros.h>

namespace location_component
{

  DetectAGV::DetectAGV() : mPrevDetectedAGV(), mCapturedFrame(0, 0, CV_8UC3)
  {
  }

  DetectAGV::~DetectAGV()
  {
  }

  void DetectAGV::detectFrame(const cv::Mat& aFrame, cv::Mat& aDisplayName)
  {
    boost::optional<DetectedAGV> lDetectedAGV = detect(aFrame);
    if (mCapturedFrame.cols == 0)
    {
      mCapturedFrame = cv::Mat(aFrame.rows, aFrame.cols, CV_8UC3);
    }
    cv::Mat lLeftDispFrame;
    aFrame.copyTo(lLeftDispFrame);
    if (lDetectedAGV)
    {
      for (const cv::Point& lCorner : (*lDetectedAGV).mCorners)
      {
        cv::circle(lLeftDispFrame, lCorner, 10, cv::Scalar(0, 0, 255),
                   CV_FILLED);
      }
      cv::circle(lLeftDispFrame, (*lDetectedAGV).mMidpoint, 10,
                 cv::Scalar(0, 255, 0), CV_FILLED, 8, 0);

      if (mPrevDetectedAGV)
      {
        bool lCapture = false;
        // Check if the AGV has moved from one side of the aFrame to another.
        if ((*lDetectedAGV).mMidpoint.x < aFrame.cols / 2 &&
            (*mPrevDetectedAGV).mMidpoint.x >= aFrame.cols / 2)
        {
          ROS_DEBUG_STREAM("AGV IS GOING LEFT");
          lCapture = true;
        }
        if ((*mPrevDetectedAGV).mMidpoint.x < aFrame.cols / 2 &&
            (*lDetectedAGV).mMidpoint.x >= aFrame.cols / 2)
        {
          ROS_DEBUG_STREAM("AGV IS GOING RIGHT");
          lCapture = true;
        }

        if (lCapture)
        {
          CupScanner lCupScanner;
          std::vector<DetectedCup> lDetectedCups =
              lCupScanner.detectCups((*lDetectedAGV).agvFrame);

          aFrame.copyTo(mCapturedFrame);

          cv::Mat displayCups;
          (*lDetectedAGV).agvFrame.copyTo(displayCups);

          PosCalculation lPosCalculator;

          for (const auto& detectedCup : lDetectedCups)
          {
            cv::circle(displayCups, detectedCup.mMidpoint, 10,
                       cv::Scalar(255, 0, 0), 0);
            ROS_INFO_STREAM(
                "Cup found at: " << lPosCalculator.calculateCupLocation(
                    (*lDetectedAGV).mMidpoint,
                    cv::Size(aFrame.cols, aFrame.rows), detectedCup.mMidpoint,
                    cv::Size(displayCups.cols, displayCups.rows)));
          }

          imshow("display ", displayCups);

          ROS_INFO_STREAM(
              "AGV found at: " << lPosCalculator.calculateAGVLocation(
                  (*lDetectedAGV).mMidpoint,
                  cv::Size(aFrame.cols, aFrame.rows)));
        }
      }
    }
    cv::hconcat(lLeftDispFrame, mCapturedFrame, aDisplayName);
    mPrevDetectedAGV = lDetectedAGV;
  }

  boost::optional<DetectedAGV> DetectAGV::detect(const cv::Mat& aFrame) const
  {
    cv::Mat lDisFrame;
    std::vector<std::vector<cv::Point>> lContours(1);

    getContoursMat(aFrame, lContours);

    cv::Rect lBoundRect;

    // Getting the middle point of the rect and draw this point
    if (lContours.at(0).size() == cCornersOfObject)
    {
      DetectedAGV lDetectedAGV;
      lBoundRect = boundingRect(lContours.at(0));
      // The corners of the AGV.
      std::vector<cv::Point2f> lAGVCorners;
      // The corners of the bounding rectangle around the AGV.
      std::vector<cv::Point2f> lEstimatedSquare;

      for (size_t idx = 0; idx < cCornersOfObject; ++idx)
      {
        lAGVCorners.push_back(cv::Point2f(( float )lContours.at(0).at(idx).x,
                                          ( float )lContours.at(0).at(idx).y));

        lDetectedAGV.mCorners.push_back(lContours.at(0).at(idx));
      }

      lEstimatedSquare.push_back(
          cv::Point2f(( float )lBoundRect.x, ( float )lBoundRect.y));
      lEstimatedSquare.push_back(cv::Point2f(
          ( float )lBoundRect.x, ( float )(lBoundRect.y + lBoundRect.height)));
      lEstimatedSquare.push_back(
          cv::Point2f(( float )(lBoundRect.x + lBoundRect.width),
                      ( float )(lBoundRect.y + lBoundRect.height)));
      lEstimatedSquare.push_back(cv::Point2f(
          ( float )(lBoundRect.x + lBoundRect.width), ( float )lBoundRect.y));

      cv::Mat lTransmtx =
          getPerspectiveTransform(lAGVCorners, lEstimatedSquare);

      makePerspectiveCorrection(lTransmtx, aFrame, lDisFrame);

      std::vector<std::vector<cv::Point>> lContours(1);
      getContoursMat(lDisFrame, lContours);

      if (lContours.at(0).size() == cCornersOfObject)
      {
        lDetectedAGV.agvFrame = lDisFrame(lBoundRect);

        std::vector<cv::Point2f> lPoints, lPointInOriginalPerspective;
        lPoints.push_back(getMidPoint(lContours.at(0)));

        cv::perspectiveTransform(lPoints, lPointInOriginalPerspective,
                                 lTransmtx.inv());

        lDetectedAGV.mMidpoint = lPointInOriginalPerspective.at(0);
      }

      return lDetectedAGV;
    }
    else
    {
      return boost::optional<DetectedAGV>();
    }
  }

  void DetectAGV::makePerspectiveCorrection(const cv::Mat& aTransmtx,
                                            const cv::Mat& aSourceMat,
                                            cv::Mat& aDist) const
  {
    aDist = cv::Mat::zeros(aSourceMat.rows, aSourceMat.cols, CV_8UC3);
    warpPerspective(aSourceMat, aDist, aTransmtx, aSourceMat.size());
  }

  void DetectAGV::getContoursMat(
      const cv::Mat& aSourceMat,
      std::vector<std::vector<cv::Point>>& aContoursPoly) const
  {
    std::vector<std::vector<cv::Point>> lContours;
    cv::Mat lMatDes;
    cv::inRange(aSourceMat, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 30),
                lMatDes);
    cv::findContours(lMatDes, lContours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);

    if (lContours.size() == 0)
      return;

    int lLargestArea = 0;
    int lLargestContourIndex = 0;

    for (size_t idx = 0; idx < lContours.size(); idx++)
    {
      double lArea = cv::contourArea(lContours.at(idx), false);
      if (lArea > lLargestArea)
      {
        lLargestArea = ( int )lArea;
        lLargestContourIndex = ( int )idx;
      }
    }

    // Copy the right rectengle tot contour_poly

    approxPolyDP(cv::Mat(lContours.at(lLargestContourIndex)),
                 aContoursPoly.at(0), 5, true);
  }

  cv::Point
      DetectAGV::getMidPoint(const std::vector<cv::Point>& aContours) const
  {
    unsigned int lSumX = 0;
    unsigned int lSumY = 0;

    for (size_t idx = 0; idx < aContours.size(); ++idx)
    {
      lSumX += aContours.at(idx).x;
      lSumY += aContours.at(idx).y;
    }

    unsigned int lAverageX = lSumX / ( unsigned int )aContours.size();
    unsigned int lAverageY = lSumY / ( unsigned int )aContours.size();

    return cv::Point(lAverageX, lAverageY);
  }

} // namespace location_component
