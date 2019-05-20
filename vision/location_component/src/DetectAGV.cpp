#include "DetectAGV.hpp"

DetectCup::DetectCup()
{
}

DetectCup::~DetectCup()
{
}

void DetectCup::detectFrame(const cv::Mat& frame, cv::Mat& debugFrame)
{
  boost::optional<DetectedAGV> detectedAGV = detectAGV(frame, debugFrame);
  if (detectedAGV)
  {
    for (const cv::Point& corner : (*detectedAGV).mCorners) {
      cv::circle(debugFrame, corner, 10, cv::Scalar(0, 0, 255),
                 CV_FILLED);
    }
    cv::circle(frame, (*detectedAGV).mMidpoint, 10,
               cv::Scalar(0, 255, 0), CV_FILLED, 8, 0);
  }
}

boost::optional<DetectedAGV> DetectCup::detectAGV(const cv::Mat& frame,
                                                  cv::Mat& debugFrame)
{
  cv::Mat disFrame;
  std::vector<std::vector<cv::Point>> contours(1);

  getContoursMat(frame, debugFrame, contours);

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
    cv::Mat t;
    getContoursMat(disFrame, t, contours);

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

void DetectCup::makePerspectiveCorrection(const cv::Mat& transmtx,
                                          const cv::Mat& sourceMat,
                                          cv::Mat& dist)
{
  dist = cv::Mat::zeros(sourceMat.rows, sourceMat.cols, CV_8UC3);
  warpPerspective(sourceMat, dist, transmtx, sourceMat.size());
}

void DetectCup::getContoursMat(
    const cv::Mat& sourceMat,
    cv::Mat& debugMat,
    std::vector<std::vector<cv::Point>>& contours_poly)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat matDes;
  cv::inRange(sourceMat, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 30), matDes);
  cv::Mat debugMatChs[] = { matDes, matDes, matDes };
  cv::merge(debugMatChs, 3, debugMat);
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
