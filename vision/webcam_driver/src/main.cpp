#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <vector>

namespace webcam_driver_constants
{

  const std::string cWebcamTopic = "/sensor/webcam/img_raw";
  const std::string cComponentName = "webcam_driver";
} // namespace webcam_driver_constants

int main(int argc, char** argv)
{

  ros::init(argc, argv, webcam_driver_constants::cComponentName);
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(nh);
  const std::string cTopicName = webcam_driver_constants::cWebcamTopic;
  /* image_transport::Subscriber sub = it.subscribe(cTopicName, 1,
   * imageCallback); */

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
