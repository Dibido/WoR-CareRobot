// Bring in gtest
#include <gtest/gtest.h>
#include "../include/DetectAGV.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

//Testing the fucntion getMidPoint
TEST(TestSuite, testCase1)
{
	std::vector<cv::Point> contours;
	contours.push_back(cv::Point(0, 0));
	contours.push_back(cv::Point(100, 0));
	contours.push_back(cv::Point(100, 100));
	contours.push_back(cv::Point(0, 100));


	DetectAGV d;
	cv::Point point = d.getMidPoint(contours);
  	EXPECT_EQ(point.x, point.y);
}

//Testing the contour function
TEST(TestSuite, testCase2)
{
    cv::Mat image = cv::imread("src/wor-18-19-s2/vision/location_component/unitTests/pictures/Test_picture.png", cv::IMREAD_COLOR);
	cv::Mat mat;


	DetectAGV d;
	std::vector<std::vector<cv::Point>> contours(1);
	d.getContoursMat(image,contours);

	//The square has 4 corners
	EXPECT_EQ(4, contours.at(0).size());
}

TEST(TestSuite, testCase3)
{
    cv::Mat image = cv::imread("src/wor-18-19-s2/vision/location_component/unitTests/pictures/Test_picture.png", cv::IMREAD_COLOR);
	cv::Mat mat;

	DetectAGV d;
	boost::optional<DetectedAGV> agv = d.detect(image);

	EXPECT_EQ(328, agv->mMidpoint.x);
	EXPECT_EQ(505, agv->mMidpoint.y);
}

TEST(TestSuite, testCase4)
{
    cv::Mat image = cv::imread("src/wor-18-19-s2/vision/location_component/unitTests/pictures/Test_picture.png", cv::IMREAD_COLOR);
	cv::Mat mat;

	DetectAGV d;
	boost::optional<DetectedAGV> agv = d.detect(image);

	EXPECT_EQ(328, agv->mMidpoint.x);
	EXPECT_EQ(505, agv->mMidpoint.y);
}


TEST(TestSuite, testCase5)
{
    cv::Mat image = cv::imread("src/wor-18-19-s2/vision/location_component/unitTests/pictures/Test_picture_perspective_view.png", cv::IMREAD_COLOR);
	cv::Mat mat;

	DetectAGV d;
	boost::optional<DetectedAGV> agv = d.detect(image);

	EXPECT_EQ(505, agv->mMidpoint.y);
}