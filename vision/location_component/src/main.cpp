#include "location_component/DetectAGV.hpp"
#include "location_component/PosCalculation.hpp"
#include "location_component/RosServiceCup.hpp"
#include "std_msgs/String.h"
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <vector>

std::shared_ptr<location_component::DetectAGV> d;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat srcMatrix;
    cv::Mat displayMatrix;

    cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(srcMatrix);

    d->detectUpdate(srcMatrix, displayMatrix);
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
  ros::init(argc, argv, "location_component");
  ros::NodeHandle nh;
  d = std::make_shared<location_component::DetectAGV>(nh);

  ros::Rate loop_rate(10);

  cv::namedWindow("view");
  image_transport::ImageTransport it(nh);
  const std::string cTopicName = "/sensor/webcam/img_raw";
  image_transport::Subscriber sub = it.subscribe(cTopicName, 1, imageCallback);

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  cv::destroyWindow("view");

  return 0;
}
