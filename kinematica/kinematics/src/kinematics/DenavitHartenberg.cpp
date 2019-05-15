#include "kinematics/DenavitHartenberg.hpp"
#include "kinematics/EulerAngles.hpp"
#include <math.h>
#include <ros/ros.h>

namespace kinematics
{

  DenavitHartenberg::DenavitHartenberg(const std::vector<Link>& config)
      : configuration(config)
  {
  }

  Matrix<double, 6, 1>
      DenavitHartenberg::forwardKinematicsYPR(const std::vector<double>& bigTheta,
                               std::size_t start,
                               std::size_t end) const
  {
    const auto fk = forwardKinematics(bigTheta, start, end);
    EulerAngles euler(fk);
    return Matrix<double, 6, 1>{ fk[0][3],  fk[1][3],    fk[2][3],
                                 euler.yaw, euler.pitch, euler.roll };
  }
  
  Matrix<double, 4, 4>
      DenavitHartenberg::forwardKinematics(const std::vector<double>& bigTheta,
                                           std::size_t start,
                                           std::size_t end) const
  {
    assert(bigTheta.size() <= configuration.size());
    assert(end <= configuration.size());
    if (end == start)
    {
      start = 0;
      end = configuration.size();
    }
    Matrix<double, 4, 4> result = result.identity();
    std::size_t thetaIndex = start;

    for (std::size_t i = start; i < end; ++i)
    {
      auto tf = configuration[i].transformationMatrix(bigTheta[i]);
      result = result * tf;
      if (configuration[i].getType() != Joint::STATIC){
        ++thetaIndex;
      }
    }
    return result;
  }


} // namespace kinematics