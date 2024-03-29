// Bring in gtest
#include "ImagePath.hpp"
#include "location_component/CupDetectionCalibration.hpp"
#include "location_component/PosCalculation.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

location_component::CupDetectionCalibration createTestCalibration()
{
  location_component::CupDetectionCalibration lTestCalibration(false);

  lTestCalibration.mCameraPosX_m = 0.50f;
  lTestCalibration.mCameraPosY_m = -4.0f;
  lTestCalibration.mCameraPosZ_m = 1.0f;
  // Camera FOV is 90° to make unit testing easier.
  lTestCalibration.mCameraFOV_rads = M_PI / 2.0f;
  lTestCalibration.mArmY_m = 0.0f;
  lTestCalibration.mCameraFlipX = -1.0f;
  lTestCalibration.mCameraFlipY = -1.0f;
  lTestCalibration.mCupHeight_m = 0.100f;
  lTestCalibration.mCupDiameter_m = 0.075f;
  lTestCalibration.mAGVDepth_m = 0.680f;
  lTestCalibration.mAGVWidth_m = 0.350f;
  lTestCalibration.mAGVHeight_m = 0.400f;
  return lTestCalibration;
}

const location_component::CupDetectionCalibration cTestCalibration =
    createTestCalibration();
const float cTestAGVSpeed_m_s = 0.25f;

TEST(PosCalculationSuite, AGVPositionInMiddleOfScreen)
{
  location_component::PosCalculation lPosCalculator(cTestCalibration);
  cv::Point lAGVMidpoint_px(100, 100);
  cv::Size lAGVFrameSize_px(200, 200);

  cv::Point3f lAGVPosition_m =
      lPosCalculator.calculateAGVLocation(lAGVMidpoint_px, lAGVFrameSize_px);

  EXPECT_FLOAT_EQ(lAGVPosition_m.x, cTestCalibration.mCameraPosX_m);
  EXPECT_FLOAT_EQ(lAGVPosition_m.y, cTestCalibration.mCameraPosY_m);
  EXPECT_FLOAT_EQ(lAGVPosition_m.z, cTestCalibration.mCameraPosZ_m -
                                        cTestCalibration.mAGVDepth_m);
}

TEST(PosCalculationSuite, AGVPositionRightOfScreen)
{
  location_component::PosCalculation lPosCalculator(cTestCalibration);
  cv::Point lAGVMidpoint_px(200, 100);
  cv::Size lAGVFrameSize_px(200, 200);

  cv::Point3f lAGVPosition_m =
      lPosCalculator.calculateAGVLocation(lAGVMidpoint_px, lAGVFrameSize_px);

  EXPECT_FLOAT_EQ(lAGVPosition_m.x, cTestCalibration.mCameraPosX_m);
  // If the AGV is at the edge of the screen,
  // the deviation should be the same as the height from of the camera from the
  // AGV (FOV is 90°).
  EXPECT_FLOAT_EQ(lAGVPosition_m.y, cTestCalibration.mCameraPosY_m +
                                        cTestCalibration.mAGVDepth_m *
                                            cTestCalibration.mCameraFlipY);
  EXPECT_FLOAT_EQ(lAGVPosition_m.z, cTestCalibration.mCameraPosZ_m -
                                        cTestCalibration.mAGVDepth_m);
}

TEST(PosCalculationSuite, AGVPositionBottomOfScreen)
{
  location_component::PosCalculation lPosCalculator(cTestCalibration);
  cv::Point lAGVMidpoint_px(100, 200);
  cv::Size lAGVFrameSize_px(200, 200);

  cv::Point3f lAGVPosition_m =
      lPosCalculator.calculateAGVLocation(lAGVMidpoint_px, lAGVFrameSize_px);

  EXPECT_FLOAT_EQ(lAGVPosition_m.x, cTestCalibration.mCameraPosX_m +
                                        cTestCalibration.mAGVDepth_m *
                                            cTestCalibration.mCameraFlipX);
  EXPECT_FLOAT_EQ(lAGVPosition_m.y, cTestCalibration.mCameraPosY_m);
  EXPECT_FLOAT_EQ(lAGVPosition_m.z, cTestCalibration.mCameraPosZ_m -
                                        cTestCalibration.mAGVDepth_m);
}

TEST(PosCalculationSuite, CupPositionRightOfScreen)
{
  location_component::PosCalculation lPosCalculator(cTestCalibration);
  cv::Point lCupMidpoint_px(200, 100);
  cv::Size lCupFrameSize_px(200, 200);

  cv::Point3f lCupPosition_m =
      lPosCalculator.calculateCupLocation(lCupMidpoint_px, lCupFrameSize_px);

  EXPECT_FLOAT_EQ(lCupPosition_m.x, cTestCalibration.mCameraPosX_m);
  // If the cup is at the edge of the screen,
  // the deviation should be the same as the height of the camera from the
  // cup (FOV is 90°).
  EXPECT_FLOAT_EQ(lCupPosition_m.y, cTestCalibration.mCameraPosY_m +
                                        (cTestCalibration.mAGVDepth_m -
                                         cTestCalibration.mCupHeight_m) *
                                            cTestCalibration.mCameraFlipY);
  EXPECT_FLOAT_EQ(lCupPosition_m.z, cTestCalibration.mCameraPosZ_m -
                                        (cTestCalibration.mAGVDepth_m -
                                         cTestCalibration.mCupHeight_m));
}

TEST(PosCalculationSuite, CupPositionBottomOfScreen)
{
  location_component::PosCalculation lPosCalculator(cTestCalibration);
  cv::Point lCupMidpoint_px(100, 200);
  cv::Size lCupFrameSize_px(200, 200);

  cv::Point3f lCupPosition_m =
      lPosCalculator.calculateCupLocation(lCupMidpoint_px, lCupFrameSize_px);

  // If the cup is at the edge of the screen,
  // the deviation should be the same as the height of the camera from the
  // cup (FOV is 90°).
  // This test uses EXPECT_NEAR because the loss of precision is too much for
  // EXPECT_FLOAT_EQ.
  EXPECT_NEAR(
      lCupPosition_m.x,
      cTestCalibration.mCameraPosX_m +
          (cTestCalibration.mAGVDepth_m - cTestCalibration.mCupHeight_m) *
              cTestCalibration.mCameraFlipX,
      0.00001);
  EXPECT_FLOAT_EQ(lCupPosition_m.y, cTestCalibration.mCameraPosY_m);
  EXPECT_FLOAT_EQ(lCupPosition_m.z, cTestCalibration.mCameraPosZ_m -
                                        (cTestCalibration.mAGVDepth_m -
                                         cTestCalibration.mCupHeight_m));
}

TEST(PosCalculationSuite, CupArrivalTimePrediction)
{
  location_component::PosCalculation lPosCalculator(cTestCalibration);
  lPosCalculator.setAGVSpeed_m_s(cTestAGVSpeed_m_s);
  float lCupPositionY_m = -2.0f;
  // The ROS current time is erradic in unit tests, so use a constant current
  // time.
  ros::Time lCurrentTime = ros::Time(10);

  ros::Time lCupPredictedArrivalTime =
      *(lPosCalculator.predictCupArrivalTime(lCupPositionY_m, lCurrentTime));

  // 2.0 m between the cup and the arm divided by 0.25 m/s = 8.0s until the cup
  // arrives at the robotarm.
  EXPECT_FLOAT_EQ(8.0f,
                  lCupPredictedArrivalTime.toSec() - lCurrentTime.toSec());
}
