#include "ImagePath.hpp"
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

TEST(DetectAGVSuite, CalibrateAGVFrame)
{
  cv::Mat lImage =
      cv::imread(getImagePath("Test_picture_cups.png"), cv::IMREAD_COLOR);
  cv::Mat lMat;
  lImage.copyTo(lMat);

  imshow("test", lMat);
}
