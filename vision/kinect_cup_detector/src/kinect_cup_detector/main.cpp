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
#include "kinematica_msgs/Cup.h"
#include "std_msgs/String.h"

bool gCalibrated = false;
ros::Publisher gCupPublisher;

void imageCallBack(const sensor_msgs::ImageConstPtr& aMsg);

int main(int argc, char** argv)
{
  // Setup opencv
  cv::namedWindow("result");
  // Setup ROS
  ros::init(argc, argv, "kinect_cup_detector");
  ros::NodeHandle lNodeHandle;

  gCupPublisher = lNodeHandle.advertise<kinematica_msgs::Cup>(
      environment_controller::cCupTopicName, 1000);

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
    // Draw a cirlce in the center of the image
    cv::circle(displayMatrix,
               cv::Point(displayMatrix.cols / 2, displayMatrix.rows / 2), 4,
               cv::Scalar(0, 0, 255), 2);
    // Draw a line across the center of the image
    cv::line(displayMatrix, cv::Point((displayMatrix.cols / 2), 0),
             cv::Point((displayMatrix.cols / 2), displayMatrix.rows),
             cv::Scalar(0, 0, 255), 4);
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
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat displayHSV;
  cv::Mat lColorMask;
  cv::Rect lBoundedRect;
  cv::RotatedRect lRotatedRect;
  cv::Mat mApproxImage;
  int centerX;
  int centerY;
  cv::Point2f lVertices[4];
  // Filter color
  cv::cvtColor(displayMatrix, displayHSV, CV_RGB2HSV);
  cv::inRange(displayHSV, cv::Scalar(0, 100, 100), cv::Scalar(70, 255, 255),
              lColorMask);
  // Find contours
  cv::findContours(lColorMask, contours, hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  for (unsigned int i = 0; i < contours.size(); i++)
  {
    double mEpsilonMultiply = 0.03;
    double epsilon = mEpsilonMultiply * arcLength(contours.at(i), true);
    approxPolyDP(contours.at(i), mApproxImage, epsilon, true);
    if (mApproxImage.size().height == 4)
    {
      if (contourArea(contours.at(i)) > 500)
      {
        // Get the number of pixels per cm (longest side of rectangle is 30cm)
        lRotatedRect = cv::minAreaRect(contours.at(i));
        // Get the biggest side
        double lMaxDistance = 0;
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
        double lPixelsPerCm = lMaxDistance / 29.0;
        std::cout << "lPixelsPerCm : " << lPixelsPerCm << std::endl;
        // Get the center of the rectangle in the frame
        // Calculate center
        cv::Moments currentmoments = cv::moments(contours.at(i));
        centerX = ( int )(currentmoments.m10 / currentmoments.m00);
        centerY = ( int )(currentmoments.m01 / currentmoments.m00);

        /* TODO Find Cup on rectangle
        cv::findContours(regionOfInterest, regionOfInterestContours, hierarchy,
        CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0)); for (unsigned
        int j = 0; j < regionOfInterestContours.size(); j++)
        {
          std::cout << "Contours.size : " << regionOfInterestContours.size() <<
        std::endl;

          // Calculate position and size (cup should be ~7cm)
        }*/
        // Determine position relative to the robotarm
        int distFromCenterX = (displayMatrix.cols / 2) - centerX;
        int distFromCenterY = (displayMatrix.rows / 2) - centerY;
        double distFromCenterXCM = distFromCenterX / lPixelsPerCm;
        distFromCenterXCM += 3.0;
        double distFromCenterYCM = distFromCenterY / lPixelsPerCm;
        distFromCenterYCM -= 70;
        distFromCenterYCM = -distFromCenterYCM;
        std::cout << "Centerx : " << distFromCenterXCM
                  << " CenterY : " << distFromCenterYCM << std::endl;
        // Send it to the location component
        kinematica_msgs::Cup lFoundCup;
        lFoundCup.aDepth = 0.07;
        lFoundCup.aDirection = 0;
        lFoundCup.aHeight = 0.082;
        lFoundCup.aMeasurementTime = ros::Time::now();
        lFoundCup.aSensorId = 1;
        lFoundCup.aSpeed = 0;
        lFoundCup.aWidth = 0.061;
        // Since the kinect is positioned oposite of the robotarm so X=Y and
        // Y=X.
        lFoundCup.mX_m = distFromCenterYCM / 100;
        lFoundCup.mY_m = distFromCenterXCM / 100;
        lFoundCup.mZ_m = 0.00;

        lFoundCup.timeOfArrival = ros::Time::now();
        gCupPublisher.publish(lFoundCup);
        ros::spinOnce();
      }
    }
  }
  // Show in a window
  cv::cvtColor(displayHSV, displayMatrix, cv::COLOR_HSV2RGB);
  cv::rectangle(displayMatrix, lBoundedRect.tl(), lBoundedRect.br(),
                cv::Scalar(0, 0, 255), 2, 8, 0);
  for (int i = 0; i < 4; i++)
  {
    cv::line(displayMatrix, lVertices[i], lVertices[(i + 1) % 4],
             cv::Scalar(0, 255, 0), 2);
  }
  cv::circle(displayMatrix, cv::Point(centerX, centerY), 4,
             cv::Scalar(0, 0, 255), 3);
  cv::imshow("result", displayMatrix);
  int c = cv::waitKey(10);
  if (c == 27) // Escape
  {
    std::exit(0);
  }
}
//