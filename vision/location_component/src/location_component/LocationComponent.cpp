#include "location_component/LocationComponent.hpp"

namespace location_component
{

  LocationComponent::LocationComponent(ros::NodeHandle& aNodeHandle)
      : mNodeHandle(aNodeHandle)
  {
  }

  LocationComponent::~LocationComponent()
  {
  }

  void LocationComponent::runComponent(
      location_component::CupDetectionCalibration& aCupDetectionCalibration,
      location_component::AGVFrameCalibration& aAGVFrameCalibration)
  {

    mDetectAGV = std::make_shared<location_component::DetectAGV>(
        mNodeHandle, aCupDetectionCalibration, aAGVFrameCalibration);

    // Subscribe to the AGV topic for receaving the speed
    location_component::AGVSubscriber mSubscriber(
        location_component_constants::cAGVTopic, mDetectAGV);

    ros::Rate loop_rate(location_component_constants::cLoopRate);

    image_transport::ImageTransport lIt(mNodeHandle);
    const std::string cTopicName = location_component_constants::cWebcamTopic;
    image_transport::Subscriber sub =
        lIt.subscribe(cTopicName, 1, &LocationComponent::imageCallBack, this);

    while (ros::ok())
    {
      loop_rate.sleep();
      ros::spinOnce();
    }
  }

  void LocationComponent::imageCallBack(const sensor_msgs::ImageConstPtr& aMsg)
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
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                aMsg->encoding.c_str());
    }
  }

} // namespace location_component