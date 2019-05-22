#include "location_component/DetectAGV.hpp"
#include "location_component/CupScanner.hpp"
#include <ros/ros.h>

namespace location_component
{

  DetectAGV::DetectAGV() : prevDetectedAGV(), capturedFrame(0, 0, CV_8UC3)
  {
  }

  DetectAGV::~DetectAGV()
  {
  }

  void DetectAGV::detectFrame(const cv::Mat& frame, cv::Mat& displayFrame)
  {
    boost::optional<DetectedAGV> detectedAGV = detect(frame);
    if (capturedFrame.cols == 0)
    {
      capturedFrame = cv::Mat(frame.rows, frame.cols, CV_8UC3);
    }
    cv::Mat leftDispFrame;
    frame.copyTo(leftDispFrame);
    if (detectedAGV)
    {
      for (const cv::Point& corner : (*detectedAGV).mCorners)
      {
        cv::circle(leftDispFrame, corner, 10, cv::Scalar(0, 0, 255), CV_FILLED);
      }
      cv::circle(leftDispFrame, (*detectedAGV).mMidpoint, 10,
                 cv::Scalar(0, 255, 0), CV_FILLED, 8, 0);

      if (prevDetectedAGV)
      {
        bool capture = false;
        // Check if the AGV has moved from one side of the frame to another.
        if ((*detectedAGV).mMidpoint.x < frame.cols / 2 &&
            (*prevDetectedAGV).mMidpoint.x >= frame.cols / 2)
        {
          ROS_DEBUG_STREAM("AGV IS GOING LEFT");
          capture = true;
        }
        if ((*prevDetectedAGV).mMidpoint.x < frame.cols / 2 &&
            (*detectedAGV).mMidpoint.x >= frame.cols / 2)
        {
          ROS_DEBUG_STREAM("AGV IS GOING RIGHT");
          capture = true;
        }

        if (capture)
        {
          CupScanner cupScanner;
          std::vector<DetectedCup> detectedCups = cupScanner.detectCups(frame);

          frame.copyTo(capturedFrame);

          for (const auto& detectedCup : detectedCups)
          {
            cv::circle(capturedFrame, detectedCup.mMidpoint, 10,
                       cv::Scalar(255, 0, 0), CV_FILLED);
          }
        }
      }
    }
    cv::hconcat(leftDispFrame, capturedFrame, displayFrame);
    prevDetectedAGV = detectedAGV;
  }

  boost::optional<DetectedAGV> DetectAGV::detect(const cv::Mat& frame) const
  {
    cv::Mat disFrame;
    std::vector<std::vector<cv::Point>> contours(1);

    getContoursMat(frame, contours);

    cv::Rect boundRect;

    // Getting the middle point of the rect and draw this point
    if (contours.at(0).size() == CornersOfObject)
    {
      DetectedAGV detectedAGV;
      boundRect = boundingRect(contours.at(0));
      // The corners of the AGV.
      std::vector<cv::Point2f> quad_pts;
      // The corners of the bounding rectangle around the AGV.
      std::vector<cv::Point2f> square_pts;

      for (size_t idx = 0; idx < CornersOfObject; ++idx)
      {
        quad_pts.push_back(cv::Point2f(( float )contours.at(0).at(idx).x,
                                       ( float )contours.at(0).at(idx).y));

        detectedAGV.mCorners.push_back(contours.at(0).at(idx));
      }

      square_pts.push_back(
          cv::Point2f(( float )boundRect.x, ( float )boundRect.y));
      square_pts.push_back(cv::Point2f(
          ( float )boundRect.x, ( float )(boundRect.y + boundRect.height)));
      square_pts.push_back(
          cv::Point2f(( float )(boundRect.x + boundRect.width),
                      ( float )(boundRect.y + boundRect.height)));
      square_pts.push_back(cv::Point2f(( float )(boundRect.x + boundRect.width),
                                       ( float )boundRect.y));

      cv::Mat transmtx = getPerspectiveTransform(quad_pts, square_pts);

      makePerspectiveCorrection(transmtx, frame, disFrame);

      std::vector<std::vector<cv::Point>> contours(1);
      getContoursMat(disFrame, contours);

      if (contours.at(0).size() == CornersOfObject)
      {
        std::vector<cv::Point2f> points, pointInOriginalPerspective;
        points.push_back(getMidPoint(contours.at(0)));

        cv::perspectiveTransform(points, pointInOriginalPerspective,
                                 transmtx.inv());

        detectedAGV.mMidpoint = pointInOriginalPerspective.at(0);
      }

      return detectedAGV;
    }
    else
    {
      return boost::optional<DetectedAGV>();
    }
  }

  void DetectAGV::makePerspectiveCorrection(const cv::Mat& transmtx,
                                            const cv::Mat& sourceMat,
                                            cv::Mat& dist) const
  {
    dist = cv::Mat::zeros(sourceMat.rows, sourceMat.cols, CV_8UC3);
    warpPerspective(sourceMat, dist, transmtx, sourceMat.size());
  }

  void DetectAGV::getContoursMat(
      const cv::Mat& sourceMat,
      std::vector<std::vector<cv::Point>>& contoursPoly) const
  {
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat matDes;
    cv::inRange(sourceMat, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 30),
                matDes);
    cv::findContours(matDes, contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);

    if (contours.size() == 0)
      return;

    int largestArea = 0;
    int largestContourIndex = 0;

    for (size_t idx0 = 0; idx0 < contours.size(); idx0++)
    {
      double area = contourArea(contours.at(idx0), false);
      if (area > largestArea)
      {
        largestArea = ( int )area;
        largestContourIndex = ( int )idx0;
      }
    }

    // Copy the right rectengle tot contour_poly

    approxPolyDP(cv::Mat(contours.at(largestContourIndex)), contoursPoly.at(0),
                 5, true);
  }

  cv::Point DetectAGV::getMidPoint(const std::vector<cv::Point>& contours) const
  {
    unsigned int sumX = 0;
    unsigned int sumY = 0;

    for (size_t idx = 0; idx < contours.size(); ++idx)
    {
      sumX += contours.at(idx).x;
      sumY += contours.at(idx).y;
    }

    unsigned int averageX = sumX / ( unsigned int )contours.size();
    unsigned int averageY = sumY / ( unsigned int )contours.size();

    return cv::Point(averageX, averageY);
  }

} // namespace location_component
