#include "location_component/AGVSubscriber.hpp"
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

std::shared_ptr<location_component::DetectAGV> mDetectAGV;

namespace locationComponent_constants
{

  const std::string cWebcamTopic = "/sensor/webcam/img_raw";
  const std::string cAGVTopic = "/sensor/agv";
  const std::string cComponentName = "location_component";
} // namespace locationComponent_coaTopicNamenstants

void imageCallback(const sensor_msgs::ImageConstPtr& aMsg)
{
  try
  {
    cv::Mat srcMatrix;
    cv::Mat displayMatrix;

    cv_bridge::toCvShare(aMsg, "bgr8")->image.copyTo(srcMatrix);
    mDetectAGV->detectUpdate(srcMatrix, displayMatrix);
    // Disable debug windows for now.
    /* cv::imshow("view", displayMatrix); */

    int c = cv::waitKey(10);
    if (c == 27)
    {
      std::exit(0);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", aMsg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, locationComponent_constants::cComponentName);
  ros::NodeHandle nh;
  mDetectAGV = std::make_shared<location_component::DetectAGV>(nh);
  location_component::AGVSubscriber mSubscriber(locationComponent_constants::cAGVTopic, mDetectAGV);

  ros::Rate loop_rate(10);

  // Disable debug windows for now.
  /* cv::namedWindow("view"); */
  image_transport::ImageTransport it(nh);
  const std::string cTopicName = locationComponent_constants::cWebcamTopic;
  image_transport::Subscriber sub = it.subscribe(cTopicName, 1, imageCallback);

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  // Disable debug windows for now.
  /* cv::destroyWindow("view"); */

  return 0;
}
