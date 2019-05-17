#include "kinematics/DenavitHartenberg.hpp"
#include "kinematics/EulerAngles.hpp"
#include "kinematics/JacobiMatrix.hpp"
#include "kinematics/UtilityFunctions.hpp"
#include <math.h>
#include <ros/ros.h>

namespace kinematics
{

  DenavitHartenberg::DenavitHartenberg(const std::vector<Link>& lLinkConfig)
      : mLinkConfiguration(lLinkConfig)
  {
  }

  Matrix<double, 6, 1> DenavitHartenberg::forwardKinematicsYPR(
      const std::vector<double>& lBigTheta,
      std::size_t lStart,
      std::size_t lEnd) const
  {
    const auto lEndEffector = forwardKinematics(lBigTheta, lStart, lEnd);
    EulerAngles euler(lEndEffector);
    return Matrix<double, 6, 1>{ lEndEffector[0][3], lEndEffector[1][3],
                                 lEndEffector[2][3], euler.mYaw_rad,
                                 euler.mPitch_rad,   euler.mRoll_rad };
  }

  std::vector<double> DenavitHartenberg::inverseKinematics(
      const Matrix<double, 6, 1>& lGoal,
      const std::vector<double>& lCurrentBigTheta) const
  {
    std::vector<double> lNewBigTheta(lCurrentBigTheta);
    auto lVirtualEndEffector = forwardKinematicsYPR(lNewBigTheta);
    double lBeta = cIkBeta;
    std::size_t lIterationCount = 0;
    while (transformationMatrixEquals(lGoal, lVirtualEndEffector, cIkEpsilon_m,
                                      cIkEpsilon_rad,
                                      cDhTransformPosRadSplit) == false)
    {
      ++lIterationCount;
      if (lIterationCount > cIkMaxIterations)
      {
        ROS_WARN("Aborted inverse kinematics after %u iterations",
                 lIterationCount);
        break;
      }
      Matrix<double, 6, 1> lDeltaPos = (lGoal - lVirtualEndEffector);
      const auto deltaEffector(lDeltaPos * lBeta);

      const Matrix<double, 6, 7> lJacobian(
          calculateJacobiMatrix(lNewBigTheta));
      const auto inverseJacobi(lJacobian.pseudoInverse());

      const auto lDeltaTheta(inverseJacobi * deltaEffector);
      for (std::size_t i = 0; i < lNewBigTheta.size(); ++i)
      {
        lNewBigTheta[i] += lDeltaTheta.at(i)[0];
      }
      lVirtualEndEffector = forwardKinematicsYPR(lNewBigTheta);
    }
    return lNewBigTheta;
  }

  Matrix<double, 4, 4>
      DenavitHartenberg::forwardKinematics(const std::vector<double>& lBigTheta,
                                           std::size_t lStart,
                                           std::size_t lEnd) const
  {
    assert(lBigTheta.size() <= mLinkConfiguration.size());
    assert(lEnd <= mLinkConfiguration.size());
    if (lEnd == lStart)
    {
      lStart = 0;
      lEnd = mLinkConfiguration.size();
    }
    Matrix<double, 4, 4> lResult = lResult.identity();
    std::size_t mThetaIndex = lStart;
    for (std::size_t mLinkConfigurationIndex = lStart; mLinkConfigurationIndex < lEnd;
         ++mLinkConfigurationIndex)
    {
      double mVariable;
      if (mLinkConfiguration[mLinkConfigurationIndex].getType() == eJoint::STATIC)
      {
        mVariable = 0;
      }
      else
      {
        mVariable = lBigTheta[mThetaIndex];
        ++mThetaIndex;
      }
      lResult = lResult *
               mLinkConfiguration[mLinkConfigurationIndex].transformationMatrix(mVariable);
    }
    return lResult;
  }

} // namespace kinematics