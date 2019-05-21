#include "location_component/CupScanner.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

cv::Mat grayscaleToRGB(const cv::Mat& image_grayscale)
{
  cv::Mat image_rgb;
  cv::Mat image_channels[] = { image_grayscale, image_grayscale,
                               image_grayscale };
  cv::merge(image_channels, 3, image_rgb);
  return image_rgb;
}

CupScanner::CupScanner()
{
}
CupScanner::~CupScanner()
{
}

std::vector<DetectedCup> CupScanner::scan(const cv::Mat& image, cv::Mat& display)
{
  std::vector<DetectedCup> detectedCups;

  cv::Mat image_hsv;
  cv::cvtColor(image, image_hsv, CV_BGR2HSV);
  cv::Mat image_hsv_channels[3];
  cv::split(image_hsv, image_hsv_channels);
  cv::Mat image_grayscale = image_hsv_channels[1];

  cv::Mat image_edges_raw, image_edges;
  cv::inRange(image_hsv, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 30),
              image_edges_raw);
  image_edges_raw.forEach<uchar>(
      [](uchar& s, __attribute__((unused)) const int position[]) {
        s = (uchar)(255 - s);
      });
  cv::dilate(image_edges_raw, image_edges,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

  cv::Point corners[4];
  corners[0] = cv::Point();
  corners[1] = cv::Point(0, image_edges.rows - 1);
  corners[2] = cv::Point(image_edges.cols - 1, 0);
  corners[3] = cv::Point(image_edges.cols - 1, image_edges.rows - 1);
  for (std::size_t i = 0; i < 4; i++)
  {
    cv::floodFill(image_edges, corners[i], cv::Scalar(0));
  }

  cv::Mat image_left = image;
  cv::Mat image_right = grayscaleToRGB(image_edges);

  std::vector<std::vector<cv::Point>> allContours;
  cv::findContours(image_edges, allContours, CV_RETR_CCOMP,
                   CV_CHAIN_APPROX_TC89_L1);

  drawContours( image, allContours, -1, cv::Scalar(255,0,0), 2);
  

  std::vector<std::vector<cv::Point>> contours;
  std::copy_if(allContours.begin(), allContours.end(),
               std::back_inserter(contours),
               [](const std::vector<cv::Point>& contour) mutable {
                 auto help = cv::boundingRect(contour);
                 return help.width > 100 && help.height > 100;
               });

  cv::Scalar colors[3];
  colors[0] = cv::Scalar(255, 0, 0);
  colors[1] = cv::Scalar(0, 255, 0);
  colors[2] = cv::Scalar(0, 0, 255);
  for (size_t idx = 0; idx < contours.size(); idx++)
  {
    DetectedCup detectedCup;
    detectedCup.mFilled = false;
    detectedCup.mRadius = 5.0;
    cv::Moments mu = cv::moments(contours[idx]);
    cv::Point centroid{ (int)(mu.m10 / mu.m00), (int)(mu.m01 / mu.m00) };
    detectedCup.mMidpoint = centroid;
    detectedCups.push_back(detectedCup);
  }

  cv::Mat image_display_2x;
  cv::hconcat(image_left, image_right, image_display_2x);
  cv::pyrDown(image_display_2x, display);

  return detectedCups;
}
