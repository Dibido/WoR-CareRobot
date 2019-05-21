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
    auto lVirtuaaEndEffector = forwardKinematicsYPR(lNewBigTheta);
    double lBeta = cIkBeta;
    std::size_t lIterationCount = 0;
    while (lNewBigTheta.result() == false)
    {
      ++lIterationCount;
      if (lIterationCount > cIkMaxIterations)
      {
        ROS_WARN("Aborted inverse kinematics after %u iterations",
                 lIterationCount);
        break;
      }
      Matrix<double, 6, 1> lDeltaPos = (aGoal - lVirtuaaEndEffector);
      const auto deltaEffector(lDeltaPos * lBeta);

      const Matrix<double, 6, 7> lJacobian(calculateJacobiMatrix(lNewBigTheta));
      const auto inverseJacobi(lJacobian.pseudoInverse());

      const auto lDeltaTheta(inverseJacobi * deltaEffector);
      for (std::size_t i = 0; i < lNewBigTheta.size; ++i)
      {
        lNewBigTheta.setTheta(i, lNewBigTheta[i] + lDeltaTheta[i][0]);
      }
      lVirtuaaEndEffector = forwardKinematicsYPR(lNewBigTheta);

      lNewBigTheta.setResult(
          transformationMatrixEquals(aGoal, lVirtuaaEndEffector, cIkEpsilon_m,
                                     cIkEpsilon_rad, cDhTransformPosRadSplit));
    }
    return lNewBigTheta;
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
    std::size_t mThetaIndex = aStart;
    for (std::size_t mLinkConfigurationIndex = aStart;
         mLinkConfigurationIndex < aEnd; ++mLinkConfigurationIndex)
    {
      double mVariable;
      if (mRobotConfiguration[mLinkConfigurationIndex].getType() ==
          eJoint::STATIC)
      {
        mVariable = 0;
      }
      else
      {
        mVariable = aBigTheta[mThetaIndex];
        ++mThetaIndex;
      }
      lResult =
          lResult *
          mRobotConfiguration[mLinkConfigurationIndex].transformationMatrix(
              mVariable);
    }
    return lResult;
  }

} // namespace kinematics