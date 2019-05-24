#include "location_component/DetectAGV.hpp"
#include "location_component/CupScanner.hpp"
#include "location_component/PosCalculation.hpp"
#include "location_component/RosServiceCup.hpp"
#include <cmath>
#include <ros/ros.h>

namespace location_component
{

  DetectAGV::DetectAGV(ros::NodeHandle& nh)
      : mPrevDetectedAGV(), mCapturedFrame(0, 0, CV_8UC3), rosServiceCup(nh)
  {
  }

  DetectAGV::~DetectAGV()
  {
  }

  void DetectAGV::detectUpdate(const cv::Mat& aFrame, cv::Mat& aDisplayFrame)
  {
    boost::optional<DetectedFrame> lDetectedFrame =
        detectFrame(aFrame, aDisplayFrame);
    if (lDetectedFrame)
    {
      PosCalculation lPosCalculator;
      for (const auto& detectedCup : lDetectedFrame->mDetectedCups)
      {
        cv::Point3f lCupLocation_m = lPosCalculator.calculateCupLocation(
            lDetectedFrame->mDetectedAGV.mMidpoint,
            lDetectedFrame->mAGVFrameSize, detectedCup.mMidpoint,
            lDetectedFrame->mCupFrameSize);
        ros::Time lCupPredictedArrivalTime =
            lPosCalculator.predictCupArrivalTime(lCupLocation_m.y,
                                                 ros::Time::now());

        ROS_INFO_STREAM("Cup found at: " << lCupLocation_m);
        ROS_INFO_STREAM("Current time " << ros::Time::now());
        ROS_INFO_STREAM("Cup is expected to arrive at "
                        << lCupPredictedArrivalTime);

        environment_controller::Object lObject(
            environment_controller::Position(lCupLocation_m.x, cArmY_m,
                                             lCupLocation_m.z),
            cCupHeight_m, cCupDiameter_m, cCupDiameter_m, M_PI * -0.5f,
            cAGVSpeed_m_s, ros::Time::now(), 0);

        environment_controller::Cup lCup(lObject, lCupPredictedArrivalTime);

        rosServiceCup.foundCup(lCup);
      }

      ROS_INFO_STREAM("AGV found at: " << lPosCalculator.calculateAGVLocation(
                          lDetectedFrame->mDetectedAGV.mMidpoint,
                          lDetectedFrame->mAGVFrameSize));
    }
  }

  boost::optional<DetectedFrame> DetectAGV::detectFrame(const cv::Mat& aFrame,
                                                        cv::Mat& aDisplayFrame)
  {
    boost::optional<DetectedFrame> lDetectedFrame{};
    boost::optional<DetectedAGV> lDetectedAGV = detect(aFrame);
    if (mCapturedFrame.cols == 0)
    {
      mCapturedFrame = cv::Mat(aFrame.rows, aFrame.cols, CV_8UC3);
    }
    cv::Mat lLeftDispFrame;
    aFrame.copyTo(lLeftDispFrame);
    if (lDetectedAGV)
    {
      for (const cv::Point& lCorner : lDetectedAGV->mCorners)
      {
        cv::circle(lLeftDispFrame, lCorner, 10, cv::Scalar(0, 0, 255),
                   CV_FILLED);
      }
      cv::circle(lLeftDispFrame, lDetectedAGV->mMidpoint, 10,
                 cv::Scalar(0, 255, 0), CV_FILLED, 8, 0);

      if (mPrevDetectedAGV)
      {
        bool lCapture = false;
        // Check if the AGV has moved from one side of the aFrame to another.
        if (lDetectedAGV->mMidpoint.x < aFrame.cols / 2 &&
            mPrevDetectedAGV->mMidpoint.x >= aFrame.cols / 2)
        {
          ROS_DEBUG_STREAM("AGV IS GOING LEFT");
          lCapture = true;
        }
        if (mPrevDetectedAGV->mMidpoint.x < aFrame.cols / 2 &&
            lDetectedAGV->mMidpoint.x >= aFrame.cols / 2)
        {
          ROS_DEBUG_STREAM("AGV IS GOING RIGHT");
          lCapture = true;
        }

        if (lCapture)
        {
          CupScanner lCupScanner;
          lDetectedFrame = DetectedFrame();
          lDetectedFrame->mDetectedCups =
              lCupScanner.detectCups(lDetectedAGV->agvFrame);
          lDetectedFrame->mDetectedAGV = (*lDetectedAGV);

          aFrame.copyTo(mCapturedFrame);

          cv::Mat lDisplayCups;
          lDetectedAGV->agvFrame.copyTo(lDisplayCups);

          lDetectedFrame->mCupFrameSize =
              cv::Size(lDisplayCups.cols, lDisplayCups.rows);
          lDetectedFrame->mAGVFrameSize = cv::Size(aFrame.cols, aFrame.rows);

          for (const auto& detectedCup : lDetectedFrame->mDetectedCups)
          {
            cv::circle(lDisplayCups, detectedCup.mMidpoint, 10,
                       cv::Scalar(255, 0, 0), 0);
          }

          /* imshow("display ", lDisplayCups); */
        }
      }
    }
    cv::hconcat(lLeftDispFrame, mCapturedFrame, aDisplayFrame);
    mPrevDetectedAGV = lDetectedAGV;

    return lDetectedFrame;
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
