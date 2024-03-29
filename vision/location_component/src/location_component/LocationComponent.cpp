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
    location_component::AGVSubscriber lAGVSubscriber(
        location_component_constants::cAGVTopic, mDetectAGV);

    // Subscribe to the goal topic for receaving the user prefrence
    location_component::GoalSubscriberLocationComp lGoalSubscriber("/goal",
                                                                   mDetectAGV);

    image_transport::ImageTransport lIt(mNodeHandle);
    const std::string cTopicName = location_component_constants::cWebcamTopic;
    image_transport::Subscriber sub =
        lIt.subscribe(cTopicName, 1, &LocationComponent::imageCallBack, this);

    ros::spin();
  }

  void LocationComponent::imageCallBack(const sensor_msgs::ImageConstPtr& aMsg)
  {
    try
    {
      cv::Mat srcMatrix;
      cv::Mat displayMatrix;

      cv_bridge::toCvShare(aMsg, "bgr8")->image.copyTo(srcMatrix);
      srcMatrix.copyTo(displayMatrix);

      if (srcMatrix.rows > 0 && srcMatrix.cols > 0)
      {
        mDetectAGV->detectUpdate(srcMatrix, displayMatrix);
      }

      int lwaitKey = cv::waitKey(10);
      if (lwaitKey == 27)
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