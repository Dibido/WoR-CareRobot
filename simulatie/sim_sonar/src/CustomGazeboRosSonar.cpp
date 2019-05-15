
#include "sim_sonar/CustomGazeboRosSonar.hpp"

namespace simulation
{
namespace sonarplugin
{
GZ_REGISTER_SENSOR_PLUGIN(CustomGazeboRosSonar);

void CustomGazeboRosSonar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  GazeboRosSonar::Load(_parent, _sdf);
  frame_name_ = "/" + robot_namespace_;
}

}  // namespace sonarplugin
}  // namespace simulation