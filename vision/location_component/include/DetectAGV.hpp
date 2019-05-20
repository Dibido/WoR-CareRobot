#ifndef DETECTCUP_HPP
#define DETECTCUP_HPP

#include <iostream>
#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <boost/optional.hpp>

const unsigned int CornersOfObject = 4;
const unsigned int CenterMargin = 50; // Pixels
const float AngleMargin = 10;
const float AngleRect = -90;

struct DetectedAGV {
  std::vector<cv::Point> mCorners;
  cv::Point mMidpoint;
};

class DetectCup
{
    public:
  DetectCup();
  ~DetectCup();

  void detectFrame(const cv::Mat& frame, cv::Mat& displayFrame);

  boost::optional<DetectedAGV> detectAGV(const cv::Mat& frame);

  void makePerspectiveCorrection(const cv::Mat& transmtx,
                                 const cv::Mat& sourceMat,
                                 cv::Mat& dist);

  void getContoursMat(const cv::Mat& sourceMat,
                      std::vector<std::vector<cv::Point>>& contours);
  cv::Point getMidPoint(std::vector<cv::Point>& contours);

    private:
  boost::optional<DetectedAGV> prevDetectedAGV;
  cv::Mat capturedFrame;
};

#endif
