#ifndef LOCATION_COMPONENT_HPP
#define LOCATION_COMPONENT_HPP

#include "location_component/AGVSubscriber.hpp"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <vector>

namespace location_component
{
  /**
   * @brief The topic names for receiving webcam frames and AGV positions. Also used to set the component name
   *
   */
  namespace location_component_constants
  {
    const std::string cWebcamTopic = "/sensor/webcam/img_raw";
    const std::string cAGVTopic = "/sensor/agv";
    const std::string cComponentName = "location_component";
    const unsigned int cLoopRate = 10;
  } // namespace location_component_constants

  class LocationComponent
  {
      public:
    /**
     * @brief Construct a new Location Component object
     *
     * @param aNodeHandle - The nodehandler used for the ros topic
     */
    LocationComponent(ros::NodeHandle& aNodeHandle);
    ~LocationComponent();

    /**
     * @brief The main run function of the application. This is a blocking function!
     *
     * @param aCupDetectionCalibration - Config values for to determining what a cup is
     * @param aAGVFrameCalibration - config values for determining what an AGV
     * 
     */
    void runComponent(
        location_component::CupDetectionCalibration& aCupDetectionCalibration,
        location_component::AGVFrameCalibration& aAGVFrameCalibration);

    /**
     * @brief This function will be called if the topic receives a new frame.
     *
     * @param aMsg - The messages that will be converted to a frame
     */
    void imageCallBack(const sensor_msgs::ImageConstPtr& aMsg);

      private:
    /**
     * @brief A shared pointer of the AGV object for the different components that make use of it
     *
     */
    std::shared_ptr<location_component::DetectAGV> mDetectAGV;

    /**
     * @brief The node handler used for the different ros topics
     * 
     */
    ros::NodeHandle& mNodeHandle;
  };

} // namespace location_component

#endif