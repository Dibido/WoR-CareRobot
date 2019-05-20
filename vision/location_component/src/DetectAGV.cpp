#include "DetectAGV.hpp"
#include "CupScanner.hpp"
#include <ros/ros.h>

DetectAGV::DetectAGV() : prevDetectedAGV(), capturedFrame(0, 0, CV_8UC3)
{
}

DetectAGV::~DetectAGV()
{
}

void DetectAGV::detectFrame(const cv::Mat& frame, cv::Mat& displayFrame)
{
  boost::optional<DetectedAGV> detectedAGV = detect(frame);
  if (capturedFrame.cols == 0) {
    capturedFrame = cv::Mat(frame.rows, frame.cols, CV_8UC3);
  }
  if (capturedFrame.type() != frame.type()) {
    ROS_INFO_STREAM("AFOIJDOIJFDSO");
  }
  cv::Mat leftDispFrame;
  frame.copyTo(leftDispFrame);
  if (detectedAGV)
  {
    for (const cv::Point& corner : (*detectedAGV).mCorners)
    {
      cv::circle(leftDispFrame, corner, 10, cv::Scalar(0, 0, 255), CV_FILLED);
    }
    cv::circle(leftDispFrame, (*detectedAGV).mMidpoint, 10, cv::Scalar(0, 255, 0),
               CV_FILLED, 8, 0);

    if (prevDetectedAGV)
    {
      bool capture = false;
      if ((*detectedAGV).mMidpoint.x < frame.cols / 2 &&
          (*prevDetectedAGV).mMidpoint.x >= frame.cols / 2)
      {
        ROS_INFO_STREAM("GOING LEFT");
        capture = true;
      }
      if ((*prevDetectedAGV).mMidpoint.x < frame.cols / 2 &&
          (*detectedAGV).mMidpoint.x >= frame.cols / 2)
      {
        ROS_INFO_STREAM("GOING RIGHT");
        capture = true;
      }

      if (capture)
      {
        CupScanner cupScanner;
          ROS_INFO_STREAM("AAAA");
        cv::Mat t;
        std::vector<DetectedCup> detectedCups =
            cupScanner.scan(frame, t);

        frame.copyTo(capturedFrame);

        for (const auto& detectedCup : detectedCups)
        {
          ROS_INFO_STREAM("BBBB");
          cv::circle(capturedFrame, detectedCup.mMidpoint, 10,
                     cv::Scalar(255, 0, 0), CV_FILLED);
        }
      }
    }
  }
  cv::hconcat(leftDispFrame, capturedFrame, displayFrame);
  prevDetectedAGV = detectedAGV;
}

boost::optional<DetectedAGV> DetectAGV::detect(const cv::Mat& frame)
{
  cv::Mat disFrame;
  std::vector<std::vector<cv::Point>> contours(1);

  getContoursMat(frame, contours);

  cv::Rect boundRect;

  // Getting the middle point of the rect and draw this point
  if (contours.size() > 0 && contours[0].size() == 4)
  {
    DetectedAGV detectedAGV;
    boundRect = boundingRect(contours.at(0));
    std::vector<cv::Point2f> quad_pts;
    std::vector<cv::Point2f> square_pts;

    for (size_t idx = 0; idx < CornersOfObject; ++idx)
    {
      quad_pts.push_back(
          cv::Point2f(contours.at(0).at(idx).x, contours.at(0).at(idx).y));

      detectedAGV.mCorners.push_back(contours.at(0).at(idx));
    }

    square_pts.push_back(cv::Point2f(boundRect.x, boundRect.y));
    square_pts.push_back(
        cv::Point2f(boundRect.x, boundRect.y + boundRect.height));
    square_pts.push_back(cv::Point2f(boundRect.x + boundRect.width,
                                     boundRect.y + boundRect.height));
    square_pts.push_back(
        cv::Point2f(boundRect.x + boundRect.width, boundRect.y));

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
                                          cv::Mat& dist)
{
  dist = cv::Mat::zeros(sourceMat.rows, sourceMat.cols, CV_8UC3);
  warpPerspective(sourceMat, dist, transmtx, sourceMat.size());
}

void DetectAGV::getContoursMat(
    const cv::Mat& sourceMat,
    std::vector<std::vector<cv::Point>>& contours_poly)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat matDes;
  cv::inRange(sourceMat, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 30), matDes);
  cv::findContours(matDes, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  if (contours.size() == 0 || contours.size() == 0)
    return;

  int largest_area = 0;
  int largest_contour_index = 0;

  for (size_t idx_0 = 0; idx_0 < contours.size(); idx_0++)
  {
    double a = contourArea(contours.at(idx_0), false);
    if (a > largest_area)
    {
      largest_area = a;
      largest_contour_index = idx_0;
    }
  }

  // Copy the right rectengle tot contour_poly

  approxPolyDP(cv::Mat(contours.at(largest_contour_index)), contours_poly.at(0),
               5, true);
}

cv::Point DetectAGV::getMidPoint(std::vector<cv::Point>& contours)
{
  unsigned int averageX = 0;
  unsigned int averageY = 0;

  for (size_t idx = 0; idx < contours.size(); ++idx)
  {
    averageX += contours.at(idx).x;
    averageY += contours.at(idx).y;
  }

  averageX /= contours.size();
  averageY /= contours.size();

  return cv::Point(averageX, averageY);
}
