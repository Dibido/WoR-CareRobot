#include "location_component/AGVSubscriber.hpp"
#include <gtest/gtest.h>

TEST(AGVSuite, setSpeed)
{
  ros::NodeHandle lNodeHandle;

  location_component::CupDetectionCalibration lCupDetectionCalibration(false);
  location_component::AGVFrameCalibration lAGVFrameCalibration(false);

  std::shared_ptr<location_component::DetectAGV> lDetectAGV;

  lDetectAGV = std::make_shared<location_component::DetectAGV>(
      lNodeHandle, lCupDetectionCalibration, lAGVFrameCalibration);

  float lTestValue = 5;

  location_component::AGVSubscriber lAGVSubscriber("test_topic", lDetectAGV);
  location_component::AGV lAGV(lTestValue);

  lAGV.speed() = lTestValue;
  lAGVSubscriber.publishAGVSpeed(lAGV);

  EXPECT_FLOAT_EQ(lTestValue, lDetectAGV->getAGVSpeed());
}