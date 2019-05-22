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
  std::vector<cv::Point> contours;
  contours.push_back(cv::Point(0, 0));
  contours.push_back(cv::Point(100, 0));
  contours.push_back(cv::Point(100, 100));
  contours.push_back(cv::Point(0, 100));

  location_component::DetectAGV d;
  cv::Point point = d.getMidPoint(contours);
  EXPECT_EQ(point.x, point.y);
}

// Testing the contour function
TEST(DetectAGVSuite, ContoursMatSize)
{
  cv::Mat image =
      cv::imread(getImagePath("Test_picture_agv.png"), cv::IMREAD_COLOR);
  cv::Mat mat;

  location_component::DetectAGV d;
  std::vector<std::vector<cv::Point>> contours(1);
  d.getContoursMat(image, contours);

  // The square has 4 corners
  EXPECT_EQ(4, contours.at(0).size());
}

TEST(DetectAGVSuite, DetectAGVPosition)
{
  cv::Mat image =
      cv::imread(getImagePath("Test_picture_agv.png"), cv::IMREAD_COLOR);
  cv::Mat mat;

  location_component::DetectAGV d;
  boost::optional<location_component::DetectedAGV> agv = d.detect(image);

  EXPECT_EQ(328, agv->mMidpoint.x);
  EXPECT_EQ(505, agv->mMidpoint.y);
}

TEST(DetectAGVSuite, DetectAGVPositionPerspective)
{
  cv::Mat image = cv::imread(
      getImagePath("Test_picture_agv_perspective_view.png"), cv::IMREAD_COLOR);
  cv::Mat mat;

  location_component::DetectAGV d;
  boost::optional<location_component::DetectedAGV> agv = d.detect(image);

  EXPECT_EQ(335, agv->mMidpoint.x);
  EXPECT_EQ(456, agv->mMidpoint.y);
}
