// Bring in gtest
#include "ImagePath.hpp"
#include "location_component/Calibration.hpp"
#include "location_component/PosCalculation.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

const location_component::Calibration cTestCalibration = {
  .mCameraPosX_m = 0.50f,
  .mCameraPosY_m = -4.0f,
  .mCameraPosZ_m = 1.0f,
  .mCameraFOV_rads = M_PI / 2.0f,
  .mArmY_m = 0.0f,
  .mCameraFlipX = -1.0f,
  .mCameraFlipY = -1.0f,
  .mCupHeight_m = 0.100f,
  .mCupDiameter_m = 0.075f,
  .mAGVDepth_m = 0.680f,
  .mAGVWidth_m = 0.350f,
  .mAGVHeight_m = 0.400f,
  .mAGVSpeed_m_s = 0.250f
};

TEST(PosCalculationSuite, AGVPositionInMiddleOfScreen)
{
  location_component::PosCalculation lPosCalculator(cTestCalibration);
  cv::Point lAGVMidpoint_px(100, 100);
  cv::Size lAGVFrameSize_px(200, 200);

  cv::Point3f lAGVPosition_m =
      lPosCalculator.calculateAGVLocation(lAGVMidpoint_px, lAGVFrameSize_px);

  EXPECT_EQ(lAGVPosition_m.x, cTestCalibration.mCameraPosX_m);
  EXPECT_EQ(lAGVPosition_m.y, cTestCalibration.mCameraPosY_m);
  EXPECT_EQ(lAGVPosition_m.z,
            cTestCalibration.mCameraPosZ_m - cTestCalibration.mAGVDepth_m);
}

TEST(PosCalculationSuite, AGVPositionRightOfScreen)
{
  location_component::PosCalculation lPosCalculator(cTestCalibration);
  cv::Point lAGVMidpoint_px(200, 100);
  cv::Size lAGVFrameSize_px(200, 200);

  cv::Point3f lAGVPosition_m =
      lPosCalculator.calculateAGVLocation(lAGVMidpoint_px, lAGVFrameSize_px);

  EXPECT_EQ(lAGVPosition_m.x, cTestCalibration.mCameraPosX_m);
  EXPECT_EQ(lAGVPosition_m.y,
            cTestCalibration.mCameraPosY_m +
                cTestCalibration.mAGVDepth_m * cTestCalibration.mCameraFlipY);
  EXPECT_EQ(lAGVPosition_m.z,
            cTestCalibration.mCameraPosZ_m - cTestCalibration.mAGVDepth_m);
}

TEST(PosCalculationSuite, AGVPositionBottomOfScreen)
{
  location_component::PosCalculation lPosCalculator(cTestCalibration);
  cv::Point lAGVMidpoint_px(100, 200);
  cv::Size lAGVFrameSize_px(200, 200);

  cv::Point3f lAGVPosition_m =
      lPosCalculator.calculateAGVLocation(lAGVMidpoint_px, lAGVFrameSize_px);

  EXPECT_EQ(lAGVPosition_m.x,
            cTestCalibration.mCameraPosX_m +
                cTestCalibration.mAGVDepth_m * cTestCalibration.mCameraFlipX);
  EXPECT_EQ(lAGVPosition_m.y, cTestCalibration.mCameraPosY_m);
  EXPECT_EQ(lAGVPosition_m.z,
            cTestCalibration.mCameraPosZ_m - cTestCalibration.mAGVDepth_m);
}

TEST(PosCalculationSuite, CupArrivalTimePrediction)
{
  location_component::PosCalculation lPosCalculator(cTestCalibration);
  float lCupPositionY_m = 2.0f;
  ros::Time lCurrentTime = ros::Time(10);

  ros::Time lCupPredictedArrivalTime =
      lPosCalculator.predictCupArrivalTime(lCupPositionY_m, lCurrentTime);

  EXPECT_EQ(8.0f, lCupPredictedArrivalTime.toSec() - lCurrentTime.toSec());
}
