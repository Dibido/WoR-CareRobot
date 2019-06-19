// Bring in gtest
#include "ImagePath.hpp"
#include "location_component/AGVFrameCalibration.hpp"
#include "location_component/CupDetectionCalibration.hpp"
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

  location_component::CupDetectionCalibration lCupDetectionCalibration(false);
  location_component::AGVFrameCalibration lAGVFrameCalibration(false);
  location_component::DetectAGV lAGVDetector(lCupDetectionCalibration,
                                             lAGVFrameCalibration);

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

  location_component::CupDetectionCalibration lCupDetectionCalibration(false);
  location_component::AGVFrameCalibration lAGVFrameCalibration(false);
  location_component::DetectAGV lAGVDetector(lCupDetectionCalibration,
                                             lAGVFrameCalibration);

  std::vector<cv::Point> lContours(1);
  lAGVDetector.getContourMat(lImage, lContours);

  // The square has 4 corners
  EXPECT_EQ(4, lContours.size());
}

TEST(DetectAGVSuite, DetectAGVPosition)
{
  cv::Mat lImage =
      cv::imread(getImagePath("Test_picture_agv.png"), cv::IMREAD_COLOR);
  cv::Mat lMat;

  location_component::CupDetectionCalibration lCupDetectionCalibration(false);
  location_component::AGVFrameCalibration lAGVFrameCalibration(false);
  location_component::DetectAGV lAGVDetector(lCupDetectionCalibration,
                                             lAGVFrameCalibration);

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

  location_component::CupDetectionCalibration lCupDetectionCalibration(false);
  location_component::AGVFrameCalibration lAGVFrameCalibration(false);
  location_component::DetectAGV lAGVDetector(lCupDetectionCalibration,
                                             lAGVFrameCalibration);

  boost::optional<location_component::DetectedAGV> lAGV =
      lAGVDetector.detect(lImage);

  EXPECT_EQ(334, lAGV->mMidpoint.x);
  EXPECT_EQ(457, lAGV->mMidpoint.y);
}

TEST(DetectAGVSuite, NoAGVSpeed)
{
  cv::Mat lImage = cv::imread(
      getImagePath("Test_picture_agv_perspective_view.png"), cv::IMREAD_COLOR);
  cv::Mat lMat;

  location_component::CupDetectionCalibration lCupDetectionCalibration(false);
  location_component::AGVFrameCalibration lAGVFrameCalibration(false);
  location_component::DetectAGV lAGVDetector(lCupDetectionCalibration,
                                             lAGVFrameCalibration);

  lAGVDetector.setDetectObject(true);
  EXPECT_EQ(0, lAGVDetector.detectUpdate(lImage, lMat));
}

TEST(DetectAGVSuite, AGVSpeed)
{
  cv::Mat lImageLeft =
      cv::imread(getImagePath("Test_picture_agv_left.jpg"), cv::IMREAD_COLOR);

  cv::Mat lImageRight =
      cv::imread(getImagePath("Test_picture_agv_right.jpg"), cv::IMREAD_COLOR);

  cv::Mat lMat;

  location_component::CupDetectionCalibration lCupDetectionCalibration(false);
  location_component::AGVFrameCalibration lAGVFrameCalibration(false);
  location_component::DetectAGV lAGVDetector(lCupDetectionCalibration,
                                             lAGVFrameCalibration);

  location_component::AGV lAGV(5.0);

  lAGVDetector.setAGVSpeed(lAGV);
  lAGVDetector.setDetectObject(true);

  EXPECT_EQ(0, lAGVDetector.detectUpdate(lImageLeft, lMat));
  EXPECT_EQ(1, lAGVDetector.detectUpdate(lImageRight, lMat));
  EXPECT_EQ(0, lAGVDetector.detectUpdate(lImageRight, lMat));
  
}


TEST(DetectAGVSuite, inactiveMode)
{
  cv::Mat lImageLeft =
      cv::imread(getImagePath("Test_picture_agv_left.jpg"), cv::IMREAD_COLOR);

  cv::Mat lImageRight =
      cv::imread(getImagePath("Test_picture_agv_right.jpg"), cv::IMREAD_COLOR);

  cv::Mat lMat;

  location_component::CupDetectionCalibration lCupDetectionCalibration(false);
  location_component::AGVFrameCalibration lAGVFrameCalibration(false);
  location_component::DetectAGV lAGVDetector(lCupDetectionCalibration,
                                             lAGVFrameCalibration);

  location_component::AGV lAGV(5.0);
  lAGVDetector.setAGVSpeed(lAGV);

  //The DetectObject is not set to true
  EXPECT_EQ(0, lAGVDetector.detectUpdate(lImageLeft, lMat));
  EXPECT_EQ(0, lAGVDetector.detectUpdate(lImageRight, lMat));  
}

TEST(DetectAGVSuite, AGVDrivingFromLeftAndViceVersa)
{
  cv::Mat lImageLeft =
      cv::imread(getImagePath("Test_picture_agv_left.jpg"), cv::IMREAD_COLOR);

  cv::Mat lImageRight =
      cv::imread(getImagePath("Test_picture_agv_right.jpg"), cv::IMREAD_COLOR);

  cv::Mat lMat;

  location_component::CupDetectionCalibration lCupDetectionCalibration(false);
  location_component::AGVFrameCalibration lAGVFrameCalibration(false);
  location_component::DetectAGV lAGVDetector(lCupDetectionCalibration,
                                             lAGVFrameCalibration);

  location_component::AGV lAGV(5.0);
  lAGVDetector.setAGVSpeed(lAGV);
  lAGVDetector.setDetectObject(true);

  EXPECT_EQ(0, lAGVDetector.detectUpdate(lImageLeft, lMat));
  EXPECT_EQ(1, lAGVDetector.detectUpdate(lImageRight, lMat));

  lAGVDetector.setDetectObject(true);
  EXPECT_EQ(1, lAGVDetector.detectUpdate(lImageLeft, lMat));
}

TEST(DetectAGVSuite, LocationComponentSetToInactiveAfterDetection)
{
  cv::Mat lImageLeft =
      cv::imread(getImagePath("Test_picture_agv_left.jpg"), cv::IMREAD_COLOR);

  cv::Mat lImageRight =
      cv::imread(getImagePath("Test_picture_agv_right.jpg"), cv::IMREAD_COLOR);

  cv::Mat lMat;

  location_component::CupDetectionCalibration lCupDetectionCalibration(false);
  location_component::AGVFrameCalibration lAGVFrameCalibration(false);
  location_component::DetectAGV lAGVDetector(lCupDetectionCalibration,
                                             lAGVFrameCalibration);

  location_component::AGV lAGV(5.0);
  lAGVDetector.setAGVSpeed(lAGV);
  lAGVDetector.setDetectObject(true);

  EXPECT_EQ(0, lAGVDetector.detectUpdate(lImageLeft, lMat));
  EXPECT_EQ(1, lAGVDetector.detectUpdate(lImageRight, lMat));

  //Checking if the application is set to inactive
  EXPECT_EQ(0, lAGVDetector.detectUpdate(lImageLeft, lMat));
}