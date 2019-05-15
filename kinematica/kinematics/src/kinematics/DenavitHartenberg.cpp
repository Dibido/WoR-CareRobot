#include "kinematics/DenavitHartenberg.hpp"
#include "kinematics/EulerAngles.hpp"
#include "kinematics/JacobiMatrix.hpp"
#include "kinematics/UtilityFunctions.hpp"
#include <math.h>
#include <ros/ros.h>

namespace kinematics
{

  DenavitHartenberg::DenavitHartenberg(const std::vector<Link>& config)
      : configuration(config)
  {
  }

  Matrix<double, 6, 1> DenavitHartenberg::forwardKinematicsYPR(
      const std::vector<double>& bigTheta,
      std::size_t start,
      std::size_t end) const
  {
    const auto fk = forwardKinematics(bigTheta, start, end);
    EulerAngles euler(fk);
    return Matrix<double, 6, 1>{ fk[0][3],  fk[1][3],    fk[2][3],
                                 euler.yaw, euler.pitch, euler.roll };
  }

  std::vector<double> DenavitHartenberg::inverseKinematics(
      const Matrix<double, 6, 1>& goal,
      const std::vector<double>& currentBigTheta) const
  {
    std::vector<double> newConfiguration(currentBigTheta);
    auto virtualEndPoint = forwardKinematicsYPR(newConfiguration);
    double beta = IK_BETA;
    std::size_t it = 0;
    while (transformationMatrixEquals(goal, virtualEndPoint, IK_POS_EPSILON,
                                      IK_RAD_EPSILON, DH_TRANSFORM_POS_RAD_SPLIT) == false)
    {
      ++it;
      if (it > IK_MAX_ITERATIONS)
      {
        ROS_WARN("Aborted inverse kinematics after %l iterations", it);
        break;
      }
      Matrix<double, 6, 1> deltaPos = (goal - virtualEndPoint);
      const auto deltaEffector(deltaPos * beta);

      const Matrix<double, 6, 7> jacobian(
          calculateJacobiMatrix(newConfiguration));
      const auto inverseJacobi(jacobian.pseudoInverse());

      const auto deltaTheta(inverseJacobi * deltaEffector);
      for (std::size_t i = 0; i < newConfiguration.size(); ++i)
      {
        newConfiguration[i] += deltaTheta.at(i)[0];
      }
      virtualEndPoint = forwardKinematicsYPR(newConfiguration);
    }
    return newConfiguration;
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
    for (std::size_t configurationIndex = start; configurationIndex < end;
         ++configurationIndex)
    {
      double variable;
      if (configuration[configurationIndex].getType() == Joint::STATIC)
      {
        variable = 0;
      }
      else
      {
        variable = bigTheta[thetaIndex];
        ++thetaIndex;
      }
      result = result *
               configuration[configurationIndex].transformationMatrix(variable);
    }
    return result;
  }

  Matrix<double, 6, 7> DenavitHartenberg::calculateJacobiMatrix(
      const std::vector<double>& bigTheta) const
  {
    assert(bigTheta.size() == 7);
    double Ct1 = std::cos(bigTheta[0]);
    double St1 = std::sin(bigTheta[0]);
    double Ct2 = std::cos(bigTheta[1]);
    double St2 = std::sin(bigTheta[1]);
    double Ct3 = std::cos(bigTheta[2]);
    double St3 = std::sin(bigTheta[2]);
    double Ct4 = std::cos(bigTheta[3]);
    double St4 = std::sin(bigTheta[3]);
    double Ct5 = std::cos(bigTheta[4]);
    double St5 = std::sin(bigTheta[4]);
    double Ct6 = std::cos(bigTheta[5]);
    double St6 = std::sin(bigTheta[5]);
    double Ct7 = std::cos(bigTheta[6]);
    double St7 = std::sin(bigTheta[6]);
    // clang-format off
    Matrix<double, 6, 7> jacobian{
      { CalcJ11(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ12(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ13(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ14(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ15(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ16(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ17(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7) },

      { CalcJ21(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ22(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ23(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ24(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ25(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ26(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ27(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7) },

      { CalcJ31(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ32(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ33(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ34(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ35(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ36(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ37(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7) },

      { CalcJ41(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ42(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ43(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ44(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ45(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ46(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ47(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7) },

      { CalcJ51(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ52(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ53(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ54(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ55(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ56(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ57(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7) },

      { CalcJ61(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ62(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ63(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ64(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ65(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ66(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7),
        CalcJ67(Ct1, St1, Ct2, St2, Ct3, St3, Ct4, St4, Ct5, St5, Ct6, St6, Ct7, St7) }
    };
    //clang-format on
    return jacobian;
  }

} // namespace kinematics