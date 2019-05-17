#ifndef DETECTCUP_HPP
#define DETECTCUP_HPP

#include <iostream>
#include <math.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

const unsigned int CornersOfObject = 4;
const unsigned int CenterMargin = 50; // Pixels
const float AngleMargin = 10;
const float AngleRect = -90;

enum class eMode
{
  PERSPECTIVE,
  CORNERCENTER,
  CENTER_OF_RECT_ESTIMATION
};

class DetectCup
{
    public:
  DetectCup();
  ~DetectCup();

  void detectFrame(cv::Mat& frame, cv::Mat& disFrame, eMode mode);

  void perspectiveCorrection(cv::Mat& frame, cv::Mat& disFrame);
  void estimatedCornerCorrection(cv::Mat& frame, cv::Mat& disFrame);
  void CornerCorrection(cv::Mat& frame, cv::Mat& disFrame);

  void makePerspectiveCorrection(const cv::Mat& transmtx,
                                 const cv::Mat& sourceMat,
                                 cv::Mat& dist);

  void getContoursMat(cv::Mat& sourceMat,
                      std::vector<std::vector<cv::Point>>& contours);
  cv::Point getMidPoint(std::vector<cv::Point>& contours);

    private:
};

#endif