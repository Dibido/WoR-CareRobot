#include "sim_webcam/WebcamPlugin.hpp"
#include <sensor_interfaces/WebcamData.h>

namespace gazebo
{
  const std::string cWebcamDataTopic = "/sensor/webcam/img_raw";
  const std::string cWebcamPublishTopic = "/sensor/webcam";

  GZ_REGISTER_SENSOR_PLUGIN(WebcamPlugin);

  void WebcamPlugin::webcamDataCallback(const sensor_msgs::ImageConstPtr& aMsg)
  {
    mWebcamPublisher =
        mRosNode->advertise<sensor_msgs::Image>(gazebo::cWebcamPublishTopic, 1);

    mWebcamPublisher.publish(aMsg);
  }

  void WebcamPlugin::Load(sensors::SensorPtr aModel, sdf::ElementPtr aSdf)
  {
    CameraPlugin::Load(aModel, aSdf);
    // The variables are set in the utils class and the CameraPlugin class
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    GazeboRosCameraUtils::Load(aModel, aSdf);

    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "WebcamPlugin", ros::init_options::NoSigintHandler);
    }

    mRosNode.reset(new ros::NodeHandle("WebcamPlugin"));

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<sensor_msgs::Image>(
            gazebo::cWebcamDataTopic, 1,
            boost::bind(&WebcamPlugin::webcamDataCallback, this, _1),
            ros::VoidPtr(), &this->mRosQueue);
    mRosSub = this->mRosNode->subscribe(so);

    this->mRosQueueThread =
        std::thread(std::bind(&WebcamPlugin::QueueThread, this));
  }

  void WebcamPlugin::QueueThread()
  {
    static const double timeout = 0.01;
    while (this->mRosNode->ok())
    {
      this->mRosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

} // namespace gazebo
