// Bring in gtest
#include <gtest/gtest.h>
#include "../include/DetectAGV.hpp"

// Declare a test
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

// Declare another test
TEST(TestSuite, testCase2)
{
		  EXPECT_EQ(1000, 1000);
}