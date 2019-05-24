#include "location_component/DetectAGV.hpp"
#include "location_component/PosCalculation.hpp"
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <vector>
#include "location_component/RosServiceCup.hpp"

location_component::DetectAGV d;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat srcMatrix;
    cv::Mat displayMatrix;

    cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(srcMatrix);

    d.detectUpdate(srcMatrix, displayMatrix);
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
  location_component::RosServiceCup test(nh);

  environment_controller::Object object(environment_controller::Position(0,0,0), 0,0,0,0,0,ros::Time(2000000000),0);

  environment_controller::Cup cup(object,ros::Time(2000000000));

  test.foundCup(cup);
  
  location_component::PosCalculation pos;


  /* std::cout << pos.calculateAGVLocation(cv::Point(200, 100), cv::Size(400,
   * 400)) << std::endl; */

  cv::namedWindow("view");
  image_transport::ImageTransport it(nh);
  const std::string cTopicName = "/sensor/webcam/img_raw";
  image_transport::Subscriber sub = it.subscribe(cTopicName, 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");

  return 0;
}
