#include "DetectCup.hpp"

DetectCup::DetectCup()
{
}

DetectCup::~DetectCup()
{
}

void DetectCup::detectFrame(cv::Mat& frame, cv::Mat& disFrame, eMode mode)
{
  switch (mode)
  {
  case eMode::PERSPECTIVE:
    perspectiveCorrection(frame, disFrame);
    break;
  case eMode::CENTER_OF_RECT_ESTIMATION:
    estimatedCornerCorrection(frame, disFrame);
    break;
  case eMode::CORNERCENTER:
    CornerCorrection(frame, disFrame);
    break;
  }
}

void DetectCup::perspectiveCorrection(cv::Mat& frame, cv::Mat& disFrame)
{
  std::vector<std::vector<cv::Point>> contours(1);

  getContoursMat(frame, contours);

  cv::Rect boundRect;

  if (contours.size() > 0)
    boundRect = boundingRect(contours.at(0));

  // Getting the middle point of the rect and draw this point
  if (contours[0].size() == 4)
  {
    std::vector<cv::Point2f> quad_pts;
    std::vector<cv::Point2f> squre_pts;

    for (size_t idx = 0; idx < CornersOfObject; ++idx)
      quad_pts.push_back(
          cv::Point2f(contours.at(0).at(idx).x, contours.at(0).at(idx).y));

    squre_pts.push_back(cv::Point2f(boundRect.x, boundRect.y));
    squre_pts.push_back(
        cv::Point2f(boundRect.x, boundRect.y + boundRect.height));
    squre_pts.push_back(cv::Point2f(boundRect.x + boundRect.width,
                                    boundRect.y + boundRect.height));
    squre_pts.push_back(
        cv::Point2f(boundRect.x + boundRect.width, boundRect.y));

    cv::Mat transmtx = getPerspectiveTransform(quad_pts, squre_pts);

    makePerspectiveCorrection(transmtx, frame, disFrame);

    std::vector<std::vector<cv::Point>> contours(1);
    getContoursMat(disFrame, contours);

    if (contours.at(0).size() == CornersOfObject)
    {
      std::vector<cv::Point2f> points, pointInOriginalPerspective;
      points.push_back(getMidPoint(contours.at(0)));

      cv::perspectiveTransform(points, pointInOriginalPerspective,
                               transmtx.inv());

      circle(frame, pointInOriginalPerspective.at(0), 10, cv::Scalar(0, 255, 0),
             CV_FILLED, 8, 0);
    }
  }
}

void DetectCup::estimatedCornerCorrection(cv::Mat& frame, cv::Mat& disFrame)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::inRange(frame, cv::Scalar(0, 0, 0, 0), cv::Scalar(200, 190, 250, 0),
              disFrame);
  cv::findContours(disFrame, contours, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_SIMPLE);
  frame.copyTo(disFrame);

  for (size_t idx_0 = 0; idx_0 < contours.size(); idx_0++)
  {
    cv::RotatedRect rotatedRect = cv::minAreaRect(contours[idx_0]);

    cv::Point2f rect_points[CornersOfObject];
    rotatedRect.points(rect_points);

    double angleToPoint = rotatedRect.angle;
    if (rotatedRect.size.width > rotatedRect.size.height)
      angleToPoint -= 90;

    cv::Point2f vertices[CornersOfObject];
    rotatedRect.points(vertices);
    for (size_t i = 0; i < CornersOfObject; i++)
      line(frame, vertices[i], vertices[(i + 1) % CornersOfObject],
           cv::Scalar(0, 0, 255), 2);

    // mid point van de schatting van een vierkant
    circle(frame, rotatedRect.center, 10, cv::Scalar(0, 0, 255), CV_FILLED, 8,
           0);
  }
}

void DetectCup::CornerCorrection(cv::Mat& frame, cv::Mat& disFrame)
{
  std::vector<std::vector<cv::Point>> contours(1);

  getContoursMat(frame, contours);

  for (size_t idx = 0; idx < contours.at(0).size(); ++idx)
  {
    circle(frame, cv::Point(contours.at(0).at(idx).x, contours.at(0).at(idx).y),
           10, cv::Scalar(0, 0, 255), CV_FILLED, 8, 0);
  }

  circle(frame, getMidPoint(contours.at(0)), 10, cv::Scalar(0, 0, 255),
         CV_FILLED, 8, 0);
}

void DetectCup::makePerspectiveCorrection(const cv::Mat& transmtx,
                                          const cv::Mat& sourceMat,
                                          cv::Mat& dist)
{
  dist = cv::Mat::zeros(sourceMat.rows, sourceMat.cols, CV_8UC3);
  warpPerspective(sourceMat, dist, transmtx, sourceMat.size());
}

void DetectCup::getContoursMat(
    cv::Mat& sourceMat,
    std::vector<std::vector<cv::Point>>& contours_poly)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat matDes;
  cv::inRange(sourceMat, cv::Scalar(0, 0, 0, 0), cv::Scalar(255, 150, 130, 0),
              matDes);
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

cv::Point DetectCup::getMidPoint(std::vector<cv::Point>& contours)
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