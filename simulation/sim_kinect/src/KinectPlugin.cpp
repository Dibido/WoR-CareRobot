#include "sim_kinect/KinectPlugin.hpp"

namespace gazebo
{
  const std::string cAgvPublishTopic = "/sensor/kinect/obstacles";

  void KinectPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
  {
    GazeboRosOpenniKinect::Load(sensor, sdf);
  }

  void KinectPlugin::passObstacles(
      const environment_controller::Obstacles& aObstacles)
  {
  }

  GZ_REGISTER_SENSOR_PLUGIN(KinectPlugin);
} // namespace gazebo
