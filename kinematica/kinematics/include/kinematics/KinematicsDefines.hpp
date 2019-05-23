#ifndef KINEMATICS_DEFINES_HPP
#define KINEMATICS_DEFINES_HPP
#include <array>
#include <cmath>

namespace kinematics
{
  const std::size_t cKinematicsDoF =
      7; /// Number of degrees of freedom of the robotarm
  const std::size_t cRobotConfigurationJoints =
      cKinematicsDoF + 1; /// Number of Links including
  const double cIkBeta = 0.5;
  const double cIkEpsilon_m = 0.000001;
  const double cIkEpsilon_rad = M_PI / 180 * 3;
  const std::size_t cIkMaxIterations = 1000;
  const std::size_t cDhTransformPosRadSplit = 3;

  const std::array<std::size_t, 1> cInvertedJoints = {
    6
  }; /// All joints in this array must be inverted before and after
     /// running inverseKinematics,

} // namespace kinematics

#endif // KINEMATICS_DEFINES_HPP
