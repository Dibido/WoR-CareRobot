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

void imageCallBack(const sensor_msgs::ImageConstPtr& aMsg);

int main(int argc, char** argv)
{
  // Setup opencv
  cv::namedWindow("view");
  cv::namedWindow("Contours", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("result");
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
  try
  {
    cv::Mat displayMatrix;

    cv_bridge::toCvShare(aMsg, "bgr8")->image.copyTo(displayMatrix);
    // Disable debug windows for now.
    cv::imshow("view", displayMatrix);

    // Find the cup on the background
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat displayGrey;
    cv::Mat displayCanny;
    cv::Mat displayHSV;
    cv::Mat lColorMask;
    cv::Mat mApproxImage;
    cv::Mat regionOfInterest;
    // Filter color
    cv::cvtColor(displayMatrix, displayHSV, CV_RGB2HSV);
    cv::inRange(displayHSV, cv::Scalar(0, 100, 100), cv::Scalar(70, 255, 255),
                lColorMask);
    cv::imshow("result", lColorMask);

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
          regionOfInterest = displayHSV(boundedRect);
          // Find Cup on rectangle
          // Calculate position and size

          // cv::Scalar color = cv::Scalar(0, 0, 255);
          // cv::drawContours(displayHSV, contours, i, color, 2, 8, hierarchy,
          // 0, cv::Point());
        }
      }
    }
    /// Draw contours
    cv::Mat displayImage = cv::Mat::zeros(displayMatrix.size(), CV_8UC3);
    for (unsigned int i = 0; i < contours.size(); i++)
    {
      cv::Scalar color = cv::Scalar(0, 0, 255);
      cv::drawContours(displayImage, contours, i, color, 2, 8, hierarchy, 0,
                       cv::Point());
    }

    /// Show in a window
    cv::cvtColor(displayHSV, displayMatrix, cv::COLOR_HSV2RGB);
    // cv::imshow("Contours", displayMatrix);
    cv::imshow("Contours", regionOfInterest);

    int c = cv::waitKey(10);
    if (c == 27) // Escape
    {
      std::exit(0);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", aMsg->encoding.c_str());
  }

  //
}