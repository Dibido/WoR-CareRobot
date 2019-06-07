#include "sim_kinect/KinectPlugin.hpp"

namespace gazebo
{
  const std::string cKinectPublishTopic = "/sensor/kinect/obstacles";
  const std::string cImgSubscribeTopic = "/sensor/kinect/img_raw";

  void KinectPlugin::Load(sensors::SensorPtr aSensor, sdf::ElementPtr aSdf)
  {
    GazeboRosOpenniKinect::Load(aSensor, aSdf);
  }

  void KinectPlugin::passObstacles(
      const environment_controller::Obstacles& aObstacles)
  {
    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "AGV_model_plugin",
                ros::init_options::NoSigintHandler);
    }

    mKinectPublisher = mRosNode->advertise<kinematica_msgs::Obstacles>(
        gazebo::cKinectPublishTopic, 1);
  }

  GZ_REGISTER_SENSOR_PLUGIN(KinectPlugin);
} // namespace gazebo
