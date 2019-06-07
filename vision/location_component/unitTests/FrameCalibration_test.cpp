#include "ImagePath.hpp"
#include <gtest/gtest.h>

#include "location_component/FrameCalibration.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

TEST(DetectAGVSuite, CalibrateAGVFrame)
{
  cv::Mat lImage =
      cv::imread(getImagePath("Test_picture_frame.jpg"), cv::IMREAD_COLOR);
  cv::Mat lMat;

  location_component::AGVFrameCalibration lAGVFrameCalibration(false);
  location_component::FrameCalibration lFrameCalibration(lAGVFrameCalibration);

  lFrameCalibration.removeEverythingButAGV(lImage, lMat);

  unsigned int lTotal = lMat.rows * lMat.cols;
  unsigned int lblackPixels = lTotal - countNonZero(lMat);
  
  EXPECT_EQ(10088580, lblackPixels);
}
