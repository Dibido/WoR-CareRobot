#ifndef KINEMATICS_DEFINES_HPP
#define KINEMATICS_DEFINES_HPP
#include <cmath>

namespace kinematics
{
  const std::size_t cKinematicsDoF = 7;
  const std::size_t cRobotConfigurationJoints = cKinematicsDoF + 1;
  const double cIkBeta = 0.5;
  const double cIkEpsilon_m = 0.000001;
  const double cIkEpsilon_rad = M_PI / 180 * 3;
  const std::size_t cIkMaxIterations = 1000;
  const std::size_t cDhTransformPosRadSplit = 3;

} // namespace kinematics

#endif // KINEMATICS_DEFINES_HPP
