#ifndef CUPDETECTOR_H_
#define CUPDETECTOR_H_

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <iostream>
#include <string>
#include <vector>

#include "environment_controller/EnvironmentConsts.hpp"
#include "environment_controller/ICupProvider.hpp"
#include "environment_controller/IGoalProvider.hpp"
#include "environment_controller/Position.hpp"
#include "kinematica_msgs/Cup.h"
#include "kinematica_msgs/Goal.h"
#include "std_msgs/String.h"

namespace kinect_cupdetector
{
  // The topic to publish the cup position on
  const std::string cKinectImageTopic = "/sensor/kinect/img_raw";
  // The topic to listen for the goal on
  const std::string cGoalTopic = "/goal";
  // The colour values for the paper
  const cv::Scalar cMinHSVRectangeValues = cv::Scalar(0, 100, 100);
  const cv::Scalar cMaxHSVRectangeValues = cv::Scalar(70, 255, 255);
  const int cRectangleCornercount = 4;
  // The minimum contour size of the paper in pixels
  const int cMinRectangleContourSize = 3000;
  const double cEpsilonMultiply = 0.03;
  // The longest side of the paper
  const double cRectangleLongestSideLength = 29.0;
  // The mininmum size of the detcted cup contour in pixels
  const int cMinCupContourSize = 50;
  // The distance that the calibration dot should be from the base of the
  // robotarm
  const double cYDistanceFromRobotarm = 70.0;
  // Cup values
  const double cCupDepth = 0.07;
  const int cCupDirection = 0;
  const double cCupHeight = 0.082;
  const int cSensorId = 1;
  const int cSpeed = 0;
  const double cCupWidth = 0.061;
  const double cCupZPos = 0.02;
  const double cCupGrippperSize = 16.0;
  const double cCupGrippperWidth = 2.0;
} // namespace kinect_cupdetector

class CupDetector
{
    public:
  CupDetector(bool aDebugMode);
  ~CupDetector() = default;

    private:
  // Callbacks
  void imageCallBack(const sensor_msgs::ImageConstPtr& aMsg);
  void goalCallback(const kinematica_msgs::GoalConstPtr& aMsg);
  // Calibrate
  void calibrateKinectPosition();
  // Position values
  int mCenterPaperX;
  int mCenterPaperY;
  int mCenterCupX;
  int mCenterCupY;

  // Debug mode
  bool mDebugMode;
  // OpenCV values
  cv::Mat mDisplayMatrix;
  cv::Mat mDisplayHSV;
  cv::Mat mColorMask;

  std::vector<std::vector<cv::Point>> mImageContours;
  std::vector<cv::Vec4i> mImageHierarchy;
  cv::RotatedRect mRotatedRect;
  cv::Point2f mRectangleVertices[4];

  cv::Mat mApproxImage;
  cv::Mat mApproxImageCup;

  cv::Mat mRegionOfInterest;
  cv::Mat mRegionOfInterestHSV;
  cv::Mat mRegionOfInterestMask;
  std::vector<std::vector<cv::Point>> mRegionOfInterestContours;

  // Whether the calibration has been finished
  bool mCalibrated;
  // Whether the goal has been requested, will be set to false when the goal has
  // been sent once
  bool mSendGoal;

  // Ros values
  ros::NodeHandle mNodeHandle;
  ros::Publisher mCupPublisher;
  image_transport::Subscriber mSubscriber;
  ros::Subscriber mGoalSubscriber;
};
#endif