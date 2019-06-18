#include "location_component/DetectAGV.hpp"
#include "location_component/CupDetectionCalibration.hpp"
#include "location_component/CupScanner.hpp"
#include "location_component/RosServiceCup.hpp"
#include <cmath>
#include <ros/ros.h>

namespace location_component
{
  DetectAGV::DetectAGV(CupDetectionCalibration& aCalibration,
                       AGVFrameCalibration& aAGVFrameCalibration)
      : mPrevDetectedAGV(),
        mCapturedFrame(0, 0, CV_8UC3),
        mRosServiceCup(),
        mPosCalculator(aCalibration),
        mCalibration(aCalibration),
        mFrameCalibration(aAGVFrameCalibration)
  {
  }

  DetectAGV::DetectAGV(ros::NodeHandle& nh,
                       CupDetectionCalibration& aCalibration,
                       AGVFrameCalibration& aAGVFrameCalibration)
      : mPrevDetectedAGV(),
        mCapturedFrame(0, 0, CV_8UC3),
        mRosServiceCup(std::make_unique<RosServiceCup>(nh)),
        mPosCalculator(aCalibration),
        mCalibration(aCalibration),
        mFrameCalibration(aAGVFrameCalibration)
  {
  }

  DetectAGV::~DetectAGV()
  {
  }

  void DetectAGV::detectUpdate(const cv::Mat& aFrame, cv::Mat& aDisplayFrame)
  {
    boost::optional<DetectedFrame> lDetectedFrame;

    // If the user wants to pick up a moving cup this boolean will be true.
    if (mDetectObject || mCalibration.mDebugStatus)
    {
      lDetectedFrame = detectFrame(aFrame, aDisplayFrame);
    }

    if (lDetectedFrame)
    {
      PosCalculation lPosCalculator(mCalibration);
      for (const auto& detectedCup : lDetectedFrame->mDetectedCups)
      {
        cv::Point3f lCupLocation_m = lPosCalculator.calculateCupLocation(
            // Cup midpoint is taken from within the bounding rectangle,
            // so add the top-left corner of the bounding rectangle to the
            // position.
            detectedCup.mMidpoint +
                lDetectedFrame->mDetectedAGV.mBoundRect.tl(),
            lDetectedFrame->mAGVFrameSize);

        boost::optional<ros::Time> lpredictCupArrival =
            mPosCalculator.predictCupArrivalTime(lCupLocation_m.y,
                                                 ros::Time::now());

        ros::Time lCupPredictedArrivalTime;

        if (lpredictCupArrival)
        {
          lCupPredictedArrivalTime = *lpredictCupArrival;

          ROS_INFO_STREAM("Cup found at: " << lCupLocation_m);
          ROS_INFO_STREAM("Current time " << ros::Time::now());
          ROS_INFO_STREAM("Cup is expected to arrive at "
                          << lCupPredictedArrivalTime);
        }

        if (mRosServiceCup && lpredictCupArrival)
        {
          environment_controller::Object lObject(
              environment_controller::Position(
                  lCupLocation_m.x, mCalibration.mArmY_m, lCupLocation_m.z),
              mCalibration.mCupHeight_m, mCalibration.mCupDiameter_m,
              mCalibration.mCupDiameter_m, M_PI * -0.5f,
              mPosCalculator.getAGVSpeed_m_s(), ros::Time::now(), 0);

          environment_controller::Cup lCup(lObject, lCupPredictedArrivalTime);

          mRosServiceCup->passCup(lCup);
          mDetectObject = false;
        }
      }

      ROS_INFO_STREAM("AGV found at: " << mPosCalculator.calculateAGVLocation(
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
          CupScanner lCupScanner(mFrameCalibration);
          lDetectedFrame = DetectedFrame();
          lDetectedFrame->mDetectedCups =
              lCupScanner.detectCups(lDetectedAGV->mAGVFrame);
          lDetectedFrame->mDetectedAGV = (*lDetectedAGV);

          aFrame.copyTo(mCapturedFrame);

          cv::Mat lDisplayCups;
          lDetectedAGV->mAGVFrame.copyTo(lDisplayCups);

          lDetectedFrame->mCupFrameSize =
              cv::Size(lDisplayCups.cols, lDisplayCups.rows);
          lDetectedFrame->mAGVFrameSize = cv::Size(aFrame.cols, aFrame.rows);

          for (const auto& detectedCup : lDetectedFrame->mDetectedCups)
          {
            cv::circle(lDisplayCups, detectedCup.mMidpoint, 10,
                       cv::Scalar(255, 0, 0), 0);
          }
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
    std::vector<cv::Point> lContours(1);

    getContourMat(aFrame, lContours);

    cv::Rect lBoundRect;

    if (mCalibration.mDebugStatus)
    {
      cv::Mat lDebugFrame;
      aFrame.copyTo(lDebugFrame);
      for (std::size_t i = 0; i < lContours.size(); i++)
        circle(lDebugFrame, cvPoint(lContours[i].x, lContours[i].y), 4,
               CV_RGB(100, 0, 0), -1, 8, 0);
      imshow("DetectAGV debug window", lDebugFrame);
    }

    // Getting the middle point of the rect and draw this point
    if (lContours.size() == cCornersOfObject)
    {
      DetectedAGV lDetectedAGV;
      lBoundRect = boundingRect(lContours);

      // The corners of the AGV.
      std::vector<cv::Point2f> lAGVCorners;
      // The corners of the bounding rectangle around the AGV.
      std::vector<cv::Point2f> lEstimatedSquare;

      // Adjusting the first point of the square
      unsigned int lIdxOffsetCornersOfObject = 0;
      if (lContours.at(0).x > lContours.at(2).x)
      {
        lIdxOffsetCornersOfObject = 1;
      }

      for (size_t idx = lIdxOffsetCornersOfObject;
           idx < cCornersOfObject + lIdxOffsetCornersOfObject; ++idx)
      {
        lAGVCorners.push_back(
            cv::Point2f(( float )lContours.at(idx % cCornersOfObject).x,
                        ( float )lContours.at(idx % cCornersOfObject).y));

        lDetectedAGV.mCorners.push_back(lContours.at(idx % cCornersOfObject));
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

      std::vector<cv::Point> lContoursWithPerspectiveCorrection(1);
      getContourMat(lDisFrame, lContoursWithPerspectiveCorrection);

      cv::Mat lAGVCutOutMatrix = lDisFrame(lBoundRect);

      if (lAGVCutOutMatrix.rows == 0 || lAGVCutOutMatrix.cols == 0)
      {
        return boost::optional<DetectedAGV>();
      }

      lDetectedAGV.mAGVFrame = lAGVCutOutMatrix;

      std::vector<cv::Point2f> lPoints, lPointInOriginalPerspective;
      lPoints.push_back(getMidPoint(lContoursWithPerspectiveCorrection));

      cv::perspectiveTransform(lPoints, lPointInOriginalPerspective,
                               lTransmtx.inv());

      lDetectedAGV.mMidpoint = lPointInOriginalPerspective.at(0);
      lDetectedAGV.mBoundRect = lBoundRect;

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

  void DetectAGV::getContourMat(const cv::Mat& aSourceMat,
                                std::vector<cv::Point>& aContoursPoly) const
  {
    std::vector<std::vector<cv::Point>> lContours;
    cv::Mat lMatDes;

    mFrameCalibration.removeEverythingButAGV(aSourceMat, lMatDes);

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
    approxPolyDP(cv::Mat(lContours.at(lLargestContourIndex)), aContoursPoly,
                 cEpsilon, true);
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

  float DetectAGV::getAGVSpeed() const
  {
    return mPosCalculator.getAGVSpeed_m_s();
  }

  void DetectAGV::setAGVSpeed(const location_component::AGV& aAGV)
  {
    ROS_DEBUG_STREAM("AGV speed is updated to " + std::to_string(aAGV.speed()));
    mPosCalculator.setAGVSpeed_m_s(aAGV.speed());
  }

  void DetectAGV::setDetectObject(bool aDetectObject)
  {
    if (aDetectObject)
    {
      ROS_DEBUG_STREAM("Location component is actively searching for a cup");
    }
    else
    {
      ROS_DEBUG_STREAM(
          "Location component is not actively searching for a cup");
    }

    mDetectObject = aDetectObject;
  }

} // namespace location_component
