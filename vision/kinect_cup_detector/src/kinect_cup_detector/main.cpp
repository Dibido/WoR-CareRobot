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

bool gCalibrated = false;

void imageCallBack(const sensor_msgs::ImageConstPtr& aMsg);

int main(int argc, char** argv)
{
  // Setup opencv
  cv::namedWindow("Contours", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("result");
  cv::namedWindow("ROI");
  // Setup ROS
  ros::init(argc, argv, "kinect_cup_detector");
  ros::NodeHandle lNodeHandle;
  image_transport::ImageTransport mImageTransport(lNodeHandle);
  image_transport::Subscriber mSubscriber;
  image_transport::Publisher mPublisher;
  // Read from ROS topic
  const std::string lKinectImageTopic = "/sensor/kinect/img_raw";
  image_transport::Subscriber sub =
      mImageTransport.subscribe(lKinectImageTopic, 1, imageCallBack);
  ros::spin();
}

void imageCallBack(const sensor_msgs::ImageConstPtr& aMsg)
{
  cv::Mat displayMatrix;
  try
  {
    cv_bridge::toCvShare(aMsg, "bgr8")->image.copyTo(displayMatrix);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", aMsg->encoding.c_str());
  }

  if (!gCalibrated)
  {
    // Calibrate the center of the kinect image  to be 70 cm away from the
    // robotarm.
    cv::circle(displayMatrix,
               cv::Point(displayMatrix.cols / 2, displayMatrix.rows / 2), 4,
               cv::Scalar(0, 0, 255), 2);
    cv::imshow("result", displayMatrix);
    int c = cv::waitKey(10);
    if (c == 27) // Escape
    {
      gCalibrated = true;
    }
    else
    {
      return;
    }
  }

  // Find the cup on the background
  std::vector<std::vector<cv::Point>> contours;
  std::vector<std::vector<cv::Point>> regionOfInterestContours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat displayHSV;
  cv::Mat lColorMask;
  cv::Mat mApproxImage;
  cv::Mat regionOfInterest;
  int centerX;
  int centerY;
  // Filter color
  cv::cvtColor(displayMatrix, displayHSV, CV_RGB2HSV);
  cv::inRange(displayHSV, cv::Scalar(0, 100, 100), cv::Scalar(70, 255, 255),
              lColorMask);
  // Find contours
  cv::findContours(lColorMask, contours, hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  // Make rectangle area of interest
  for (unsigned int i = 0; i < contours.size(); i++)
  {
    double mEpsilonMultiply = 0.03;
    double epsilon = mEpsilonMultiply * arcLength(contours.at(i), true);
    approxPolyDP(contours.at(i), mApproxImage, epsilon, true);
    if (mApproxImage.size().height == 4)
    {
      if (contourArea(contours.at(i)) > 500)
      {
        // Found the rectangle, set the region of interest
        cv::Rect boundedRect = cv::boundingRect(contours.at(i));
        regionOfInterest = lColorMask(boundedRect);
        // Get the number of pixels per cm (longest side of rectangle is 30cm)
        cv::RotatedRect lRotatedRect = cv::minAreaRect(contours.at(i));
        // Get the biggest side
        double lMaxDistance = 0;
        cv::Point2f lVertices[4];
        lRotatedRect.points(lVertices);
        for (int i = 0; i < 4; i++)
        {
          double lDistance =
              ( double )cv::norm(lVertices[i] - lVertices[(i + 1) % 4]);
          if (lDistance > lMaxDistance)
          {
            lMaxDistance = ( double )lDistance;
          }
        }
        std::cout << "Pixels : " << lMaxDistance << std::endl;
        double lPixelsPerCm = lMaxDistance / 30.0;
        std::cout << "lPixelsPerCm : " << lPixelsPerCm << std::endl;
        // Get the center of the rectangle in the frame
        // Calculate center
        cv::Moments currentmoments = cv::moments(contours.at(i));
        centerX = ( int )(currentmoments.m10 / currentmoments.m00);
        centerY = ( int )(currentmoments.m01 / currentmoments.m00);
        cv::circle(displayMatrix, cv::Point(centerX, centerY), 4,
                   cv::Scalar(0, 0, 255), 3);
        cv::circle(displayMatrix, cv::Point(centerX, centerY), 4,
                   cv::Scalar(0, 0, 255), 2);
        std::cout << "Centerx : " << centerX << " CenterY : " << centerY
                  << std::endl;
        /*// Find Cup on rectangle
        cv::findContours(regionOfInterest, regionOfInterestContours, hierarchy,
        CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0)); for (unsigned
        int j = 0; j < regionOfInterestContours.size(); j++)
        {
          std::cout << "Contours.size : " << regionOfInterestContours.size() <<
        std::endl;

          // Calculate position and size (cup should be ~7cm)
        }*/
        // Send it to the location component
      }
    }
  }
  /// Draw contours
  cv::Mat displayImage = cv::Mat::zeros(regionOfInterest.size(), CV_8UC3);
  for (unsigned int i = 0; i < regionOfInterestContours.size(); i++)
  {
    cv::Scalar color = cv::Scalar(0, 0, 255);
    cv::drawContours(displayImage, regionOfInterestContours, i, color, 2, 8,
                     hierarchy, 0, cv::Point());
  }

  /// Show in a window
  cv::cvtColor(displayHSV, displayMatrix, cv::COLOR_HSV2RGB);
  cv::imshow("result", displayMatrix);
  if (displayImage.cols > 0 && displayImage.rows > 0)
  {
    cv::imshow("Contours", displayImage);
  }
  if (regionOfInterest.cols > 0 && regionOfInterest.rows > 0)
  {
    cv::imshow("ROI", regionOfInterest);
  }
  int c = cv::waitKey(10);
  if (c == 27) // Escape
  {
    std::exit(0);
  }
}
//