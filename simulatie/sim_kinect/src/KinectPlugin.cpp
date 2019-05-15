#include "sim_kinect/KinectPlugin.hpp"

namespace gazebo
{
void KinectPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  GazeboRosOpenniKinect::Load(sensor, sdf);
}

GZ_REGISTER_SENSOR_PLUGIN(KinectPlugin);
}  // namespace gazebo
