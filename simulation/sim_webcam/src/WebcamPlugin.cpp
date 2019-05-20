#include "sim_webcam/WebcamPlugin.hpp"
#include <sensor_interfaces/WebcamData.h>

const std::string cWebcamDataTopic = "/sensor/webcam/img_raw";
const std::string cWebcamPublishTopic = "/sensor/webcam";

namespace gazebo
{
  GZ_REGISTER_SENSOR_PLUGIN(WebcamPlugin);

  void WebcamPlugin::callback(const sensor_msgs::Image& aMsg)
  {
    ros::NodeHandle n;
    ros::Publisher webcamPublisher =
        n.advertise<sensor_msgs::Image>(cWebcamPublishTopic, 1000);
    sensor_interfaces::WebcamData newFrame;
    newFrame.frame = aMsg;
    webcamPublisher.publish(newFrame);
  }

  void WebcamPlugin::Load(sensors::SensorPtr aModel, sdf::ElementPtr aSdf)
  {

    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "Parser", ros::init_options::NoSigintHandler);
      ROS_FATAL_STREAM(
          "A ROS node for Gazebo has not been initialized, unable to load "
          "plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
             "the gazebo_ros package)");
      return;
    }

    CameraPlugin::Load(aModel, aSdf);

    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    // GazeboRosCameraUtils::Load(aModel, aSdf);

    rosNode = std::make_unique<ros::NodeHandle>("robot_simulation_plugin");

    // Subscribe to
    rosSub = rosNode->subscribe(cWebcamDataTopic, &WebcamPlugin::callback, this)
  }

} // namespace gazebo
