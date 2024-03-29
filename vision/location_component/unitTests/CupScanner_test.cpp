// Bring in gtest
#include "ImagePath.hpp"
#include "location_component/CupScanner.hpp"
#include "location_component/DetectAGV.hpp"
#include "location_component/FrameCalibration.hpp"

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

location_component::AGVFrameCalibration gAGVFrameCalibration(false);
location_component::FrameCalibration gFrameCalibration(gAGVFrameCalibration);
location_component::CupDetectionCalibration gCupDetectionCalibration(false);

// Testing the fucntion getMidPoint
TEST(CupScannerSuite, RecogniseMultipleCups)
{
  cv::Mat lImage =
      cv::imread(getImagePath("Test_picture_cups.png"), cv::IMREAD_COLOR);
  cv::Mat lMat;
  lImage.copyTo(lMat);

  location_component::CupScanner lScanner(gFrameCalibration,
                                          gCupDetectionCalibration);

  std::vector<location_component::DetectedCup> lCups =
      lScanner.detectCups(lImage, lMat);

  EXPECT_EQ(lCups.size(), 4);
}

TEST(CupScannerSuite, RecogniseMultipleCupsPositions)
{
  cv::Mat lImage =
      cv::imread(getImagePath("Test_picture_cups.png"), cv::IMREAD_COLOR);
  cv::Mat lMat;
  lImage.copyTo(lMat);

  location_component::CupScanner lScanner(gFrameCalibration,
                                          gCupDetectionCalibration);

  std::vector<location_component::DetectedCup> lCups =
      lScanner.detectCups(lImage, lMat);

  EXPECT_EQ(lCups.at(0).mMidpoint.x, 462);
  EXPECT_EQ(lCups.at(0).mMidpoint.y, 481);

  EXPECT_EQ(lCups.at(1).mMidpoint.x, 197);
  EXPECT_EQ(lCups.at(1).mMidpoint.y, 447);

  EXPECT_EQ(lCups.at(2).mMidpoint.x, 650);
  EXPECT_EQ(lCups.at(2).mMidpoint.y, 338);

  EXPECT_EQ(lCups.at(3).mMidpoint.x, 411);
  EXPECT_EQ(lCups.at(3).mMidpoint.y, 248);
}

TEST(CupScannerSuite, RecogniseMultipleCupsFilled)
{
  cv::Mat lImage =
      cv::imread(getImagePath("Test_picture_cups.png"), cv::IMREAD_COLOR);
  cv::Mat lMat;
  lImage.copyTo(lMat);

  location_component::CupScanner lScanner(gFrameCalibration,
                                          gCupDetectionCalibration);

  std::vector<location_component::DetectedCup> lCups =
      lScanner.detectCups(lImage, lMat);

  EXPECT_EQ(lCups.at(0).mFilled, true);

  EXPECT_EQ(lCups.at(1).mFilled, false);

  EXPECT_EQ(lCups.at(2).mFilled, true);

  EXPECT_EQ(lCups.at(3).mFilled, false);
}
