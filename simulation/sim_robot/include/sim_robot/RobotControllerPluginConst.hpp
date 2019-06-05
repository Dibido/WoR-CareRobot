#ifndef ROBOTCONTROLLERPLUGINCONSTS_HPP
#define ROBOTCONTROLLERPLUGINCONSTS_HPP
#include "types.hpp"

namespace robotcontrollerplugin
{
  const int gripperJoint = 7;
  const jointVel_t cMaxSpeedfactor = 1;
  const jointVel_t cMinSpeedfactor = 0.0;
  const jointRad_t cMaxRad = 3.7525;
  const jointRad_t cMinRad = -3.0718;
  const double maxWidthgripper = 0.08;
  const double minWidthgripper = 0.0;
} // namespace robotcontrollerplugin
#endif