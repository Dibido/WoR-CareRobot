#ifndef KINEMATICS_DEFINES_HPP
#define KINEMATICS_DEFINES_HPP
#include <array>
#include <cmath>
#include <vector>

namespace kinematics
{
  const std::size_t cKinematicsDoF =
      7; /// Number of degrees of freedom of the robotarm
  const std::size_t cRobotConfigurationJoints =
      cKinematicsDoF + 1; /// Number of Links including static links

  const std::vector<std::pair<double, double>> cBetaSelectors = { { 0.9, 0.2 },
                                                                  { 0.0,
                                                                    0.3 } };
  const double cIkEpsilon_m = 0.000001;
  const double cIkEpsilon_rad = M_PI / 180 * 5;
  const std::size_t cIkMaxIterations = 5000;
  const std::size_t cDhTransformPosRadSplit = 3;

  const std::array<std::size_t, 1> cInvertedJoints = {
    6
  }; /// All joints in this array must be inverted before and after
     /// running inverseKinematics and before running forwardKinematics in the
     /// interface IConfigurationProvider
} // namespace kinematics

#endif // KINEMATICS_DEFINES_HPP
