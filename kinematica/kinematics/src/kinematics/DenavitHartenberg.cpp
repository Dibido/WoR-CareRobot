#include "kinematics/DenavitHartenberg.hpp"
#include "kinematics/EulerAngles.hpp"
#include "kinematics/JacobiMatrix.hpp"
#include "kinematics/KinematicsDefines.hpp"
#include "kinematics/UtilityFunctions.hpp"
#include <math.h>
#include <ros/ros.h>

namespace kinematics
{

  DenavitHartenberg::DenavitHartenberg() : mRobotConfiguration()
  {
  }

  Matrix<double, 6, 1>
      DenavitHartenberg::forwardKinematicsYPR(const Configuration& aBigTheta,
                                              std::size_t aStart,
                                              std::size_t aEnd) const
  {
    const auto aEndEffector = forwardKinematics(aBigTheta, aStart, aEnd);
    EulerAngles euler(aEndEffector);
    return Matrix<double, 6, 1>{ aEndEffector[0][3], aEndEffector[1][3],
                                 aEndEffector[2][3], euler.mYaw_rad,
                                 euler.mPitch_rad,   euler.mRoll_rad };
  }

  Configuration DenavitHartenberg::inverseKinematics(
      const Matrix<double, 6, 1>& aGoal,
      const Configuration& aCurrentBigTheta) const
  {
    Configuration lNewBigTheta(aCurrentBigTheta);
    auto lVirtualEndEffector = forwardKinematicsYPR(lNewBigTheta);
    double lBeta = cIkBeta;
    std::size_t lIterationCount = 0;

    lNewBigTheta.setResult(
        transformationMatrixEquals(aGoal, lVirtualEndEffector, cIkEpsilon_m,
                                   cIkEpsilon_rad, cDhTransformPosRadSplit));
    while (lNewBigTheta.result() == false)
    {
      ++lIterationCount;
      if (lIterationCount > cIkMaxIterations)
      {
        ROS_WARN("Aborted inverse kinematics after %u iterations",
                 lIterationCount);
        break;
      }
      Matrix<double, 6, 1> lDeltaPos = (aGoal - lVirtualEndEffector);
      const auto deltaEffector(lDeltaPos * lBeta);

      const Matrix<double, 6, 7> lJacobian(calculateJacobiMatrix(lNewBigTheta));
      const auto inverseJacobi(lJacobian.pseudoInverse());

      const auto lDeltaTheta(inverseJacobi * deltaEffector);

      for (std::size_t i = 0; i < lNewBigTheta.size; ++i)
      {
        double newTheta = lNewBigTheta[i] + lDeltaTheta[i][0];
        lNewBigTheta.setTheta(i, newTheta);
      }
      lVirtualEndEffector = forwardKinematicsYPR(lNewBigTheta);

      lNewBigTheta.setResult(
          transformationMatrixEquals(aGoal, lVirtualEndEffector, cIkEpsilon_m,
                                     cIkEpsilon_rad, cDhTransformPosRadSplit));
      if (lNewBigTheta.result() == true)
      {
        if (isValidConfiguration(lNewBigTheta) == false)
        {
          randomizeConfiguration(lNewBigTheta);
          lVirtualEndEffector = forwardKinematicsYPR(lNewBigTheta);
        }
      }
    }
    return lNewBigTheta;
  }

  bool DenavitHartenberg::isValidConfiguration(
      const Configuration& aConfiguration) const
  {
    bool withinConstraints = true;

    std::size_t lThetaIndex = 0;
    for (std::size_t lRobotConfigurationIndex = 0;
         lRobotConfigurationIndex < mRobotConfiguration.size;
         ++lRobotConfigurationIndex)
    {
      if (mRobotConfiguration[lRobotConfigurationIndex].getType() ==
          eJoint::STATIC)
      {
        // Do nothing
      }
      else
      {
        if (mRobotConfiguration[lRobotConfigurationIndex].isWithinConstraints(
                aConfiguration[lThetaIndex]) == false)
        {
          withinConstraints = false;
          break;
        }
        ++lThetaIndex;
      }
    }
    return withinConstraints;
  }

  void DenavitHartenberg::randomizeConfiguration(
      Configuration& configuration) const
  {
    std::size_t lThetaIndex = 0;
    for (std::size_t lRobotConfigurationIndex = 0;
         lRobotConfigurationIndex < mRobotConfiguration.size;
         ++lRobotConfigurationIndex)
    {
      if (mRobotConfiguration[lRobotConfigurationIndex].getType() ==
          eJoint::STATIC)
      {
        // Do nothing
      }
      else
      {
#define PARTIAL_SWEEP
#ifdef PARTIAL_SWEEP
        double lCurValue = configuration[lThetaIndex];
        if (mRobotConfiguration[lRobotConfigurationIndex].isWithinConstraints(
                lCurValue) == false)
        {
#endif
          double lNewValue = mRobotConfiguration[lRobotConfigurationIndex]
                                 .generateRandomVariable();
          configuration.setTheta(lThetaIndex, lNewValue);
#ifdef PARTIAL_SWEEP
        }
#endif
        ++lThetaIndex;
      }
    }
    configuration.setResult(false);
  }

  Matrix<double, 4, 4>
      DenavitHartenberg::forwardKinematics(const Configuration& aBigTheta,
                                           std::size_t aStart,
                                           std::size_t aEnd) const
  {
    if (aEnd == aStart)
    {
      aStart = 0;
      aEnd = mRobotConfiguration.size;
    }
    Matrix<double, 4, 4> lResult = lResult.identity();
    std::size_t lThetaIndex = aStart;
    for (std::size_t lRobotConfigurationIndex = aStart;
         lRobotConfigurationIndex < aEnd; ++lRobotConfigurationIndex)
    {
      double mVariable;
      if (mRobotConfiguration[lRobotConfigurationIndex].getType() ==
          eJoint::STATIC)
      {
        mVariable = 0;
      }
      else
      {
        mVariable = aBigTheta[lThetaIndex];
        ++lThetaIndex;
      }
      lResult =
          lResult *
          mRobotConfiguration[lRobotConfigurationIndex].transformationMatrix(
              mVariable);
    }
    return lResult;
  }

} // namespace kinematics