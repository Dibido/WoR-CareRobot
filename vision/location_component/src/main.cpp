//#include "CupScanner.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/objdetect.hpp"
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "DetectCup.hpp"


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  try
  {
    DetectCup d;
    cv::Mat srcMatrix;
    cv::Mat debugMatrix;

    cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(srcMatrix);


    
    d.detectFrame(srcMatrix, debugMatrix ,eMode::CORNERCENTER);
    cv::imshow("view",  srcMatrix);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub =
      it.subscribe("/sensor/webcam/img_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");


  return 0;
}
