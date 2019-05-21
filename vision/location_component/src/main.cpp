#include "location_component/DetectAGV.hpp"
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <vector>

DetectAGV d;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat srcMatrix;
    cv::Mat displayMatrix;

    cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(srcMatrix);

    d.detectFrame(srcMatrix, displayMatrix);
    cv::imshow("view", displayMatrix);

    int c = cv::waitKey(10);
    if (c == 27)
    {
      std::exit(0);
    }
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
