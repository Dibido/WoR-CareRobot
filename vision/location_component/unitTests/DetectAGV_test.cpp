// Bring in gtest
#include "ImagePath.hpp"
#include "location_component/DetectAGV.hpp"
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

// Testing the fucntion getMidPoint
TEST(DetectAGVSuite, CalculateMidpoint)
{
  std::vector<cv::Point> lContours;
  lContours.push_back(cv::Point(0, 0));
  lContours.push_back(cv::Point(100, 0));
  lContours.push_back(cv::Point(100, 100));
  lContours.push_back(cv::Point(0, 100));

  location_component::DetectAGV lAGVDetector;
  cv::Point lPoint = lAGVDetector.getMidPoint(lContours);
  EXPECT_EQ(lPoint.x, 50);
  EXPECT_EQ(lPoint.y, 50);
}

// Testing the contour function
TEST(DetectAGVSuite, ContoursMatSize)
{
  cv::Mat lImage =
      cv::imread(getImagePath("Test_picture_agv.png"), cv::IMREAD_COLOR);
  cv::Mat lMat;

  location_component::DetectAGV lAGVDetector;
  std::vector<std::vector<cv::Point>> lContours(1);
  lAGVDetector.getContoursMat(lImage, lContours);

  // The square has 4 corners
  EXPECT_EQ(4, lContours.at(0).size());
}

TEST(DetectAGVSuite, DetectAGVPosition)
{
  cv::Mat lImage =
      cv::imread(getImagePath("Test_picture_agv.png"), cv::IMREAD_COLOR);
  cv::Mat lMat;

  location_component::DetectAGV lAGVDetector;
  boost::optional<location_component::DetectedAGV> lAGV =
      lAGVDetector.detect(lImage);

  EXPECT_EQ(328, lAGV->mMidpoint.x);
  EXPECT_EQ(505, lAGV->mMidpoint.y);
}

TEST(DetectAGVSuite, DetectAGVPositionPerspective)
{
  cv::Mat lImage = cv::imread(
      getImagePath("Test_picture_agv_perspective_view.png"), cv::IMREAD_COLOR);
  cv::Mat lMat;

  location_component::DetectAGV lAGVDetector;
  boost::optional<location_component::DetectedAGV> lAGV =
      lAGVDetector.detect(lImage);

  EXPECT_EQ(335, lAGV->mMidpoint.x);
  EXPECT_EQ(456, lAGV->mMidpoint.y);
}
