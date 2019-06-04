#ifndef LOCATION_COMPONENT_HPP
#define LOCATION_COMPONENT_HPP

#include "location_component/AGVSubscriber.hpp"
#include "location_component/Calibration.hpp"
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

namespace location_component
{
  namespace location_component_constants
  {
    const std::string cWebcamTopic = "/sensor/webcam/img_raw";
    const std::string cAGVTopic = "/sensor/agv";
    const std::string cComponentName = "location_component";
  } // namespace location_component_constants

  class LocationComponent
  {
      public:
    LocationComponent(ros::NodeHandle& aNodeHandle);
    ~LocationComponent();
    void runComponent(location_component::CupDetectionCalibration& aCupDetectionCalibration, location_component::AGVFrameCalibration& aAGVFrameCalibration);

    void imageCallBack(const sensor_msgs::ImageConstPtr& aMsg);


      private:
    std::shared_ptr<location_component::DetectAGV> mDetectAGV;
    ros::NodeHandle& mNodeHandle;
  };

} // namespace location_component

#endif