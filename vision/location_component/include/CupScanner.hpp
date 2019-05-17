#ifndef ARUCOSCANNER_HPP
#define ARUCOSCANNER_HPP

#include <opencv2/opencv.hpp>

class CupScanner
{
    public:
  CupScanner();
  ~CupScanner();
  void scan(const cv::Mat& image, cv::Mat& display);
};

#endif /* ARUCOSCANNER_HPP */
