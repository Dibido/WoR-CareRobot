#include "sim_kinect/KinectPlugin.hpp"

namespace gazebo
{
  const std::string cKinectPublishTopic = "/sensor/kinect/obstacles";
  const std::string cImgSubscribeTopic = "/sensor/kinect/img_raw";

  void KinectPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
  {
    GazeboRosOpenniKinect::Load(sensor, sdf);
  }

  void KinectPlugin::passObstacles(const environment_controller::Obstacles&)
  {
  }

  GZ_REGISTER_SENSOR_PLUGIN(KinectPlugin);
} // namespace gazebo
