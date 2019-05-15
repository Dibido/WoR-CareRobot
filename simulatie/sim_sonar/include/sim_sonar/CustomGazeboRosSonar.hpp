
#ifndef PROJECT_CUSTOMGAZEBOROSSONAR_HPP
#define PROJECT_CUSTOMGAZEBOROSSONAR_HPP

#include "GazeboRosSonar.hpp"

using namespace gazebo;

namespace simulation
{
namespace sonarplugin
{
class CustomGazeboRosSonar : public GazeboRosSonar
{
public:
  CustomGazeboRosSonar() = default;

  virtual ~CustomGazeboRosSonar() = default;

  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
};

}  // namespace sonarplugin
}  // namespace simulation

#endif
