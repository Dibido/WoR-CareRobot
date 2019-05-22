// Bring in gtest
#include "ImagePath.hpp"
#include "location_component/CupScanner.hpp"
#include "location_component/DetectAGV.hpp"
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

// Testing the fucntion getMidPoint
TEST(CupScannerSuite, RecogniseMultipleCups)
{
  cv::Mat image =
      cv::imread(getImagePath("Test_picture_cups.png"), cv::IMREAD_COLOR);
  cv::Mat mat;
  image.copyTo(mat);

  location_component::CupScanner scanner;
  std::vector<location_component::DetectedCup> cups = scanner.scan(image, mat);

  EXPECT_EQ(cups.size(), 4);
}

TEST(CupScannerSuite, RecogniseMultipleCupsPositions)
{
  cv::Mat image =
      cv::imread(getImagePath("Test_picture_cups.png"), cv::IMREAD_COLOR);
  cv::Mat mat;
  image.copyTo(mat);

  location_component::CupScanner scanner;
  std::vector<location_component::DetectedCup> cups = scanner.scan(image, mat);

  EXPECT_EQ(cups.at(0).mMidpoint.x, 462);
  EXPECT_EQ(cups.at(0).mMidpoint.y, 481);

  EXPECT_EQ(cups.at(1).mMidpoint.x, 197);
  EXPECT_EQ(cups.at(1).mMidpoint.y, 447);

  EXPECT_EQ(cups.at(2).mMidpoint.x, 650);
  EXPECT_EQ(cups.at(2).mMidpoint.y, 338);

  EXPECT_EQ(cups.at(3).mMidpoint.x, 412);
  EXPECT_EQ(cups.at(3).mMidpoint.y, 248);
}
