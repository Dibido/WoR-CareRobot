#ifndef ROBOT_CONTROLLER_PLUGIN_CONSTS_HPP
#define ROBOT_CONTROLLER_PLUGIN_CONSTS_HPP
#include "types.hpp"

namespace robotcontrollerplugin
{
  const int gripperJoint = 7;
  const jointVel_t cMaxSpeedfactor = 1;
  const jointRad_t cMaxRad = 3.7525;
  const jointRad_t cMinRad = -3.0718;
  const double cMaxWidth_m = 1;
} // namespace robotcontrollerplugin
#endif