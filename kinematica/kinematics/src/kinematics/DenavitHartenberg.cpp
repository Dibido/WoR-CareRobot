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
        ROS_WARN("Aborted inverse kinematics after %l iterations",
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

  Matrix<double, 6, 7> DenavitHartenberg::calculateJacobiMatrix(
      const std::vector<double>& lBigTheta) const
  {
    assert(lBigTheta.size() == 7);
    double lCosT1 = std::cos(lBigTheta[0]);
    double lSinT1 = std::sin(lBigTheta[0]);
    double lCosT2 = std::cos(lBigTheta[1]);
    double lSinT2 = std::sin(lBigTheta[1]);
    double lCosT3 = std::cos(lBigTheta[2]);
    double lSinT3 = std::sin(lBigTheta[2]);
    double lCosT4 = std::cos(lBigTheta[3]);
    double lSinT4 = std::sin(lBigTheta[3]);
    double lCosT5 = std::cos(lBigTheta[4]);
    double lSinT5 = std::sin(lBigTheta[4]);
    double lCosT6 = std::cos(lBigTheta[5]);
    double lSinT6 = std::sin(lBigTheta[5]);
    double lCosT7 = std::cos(lBigTheta[6]);
    double lSinT7 = std::sin(lBigTheta[6]);
    // clang-format off
    Matrix<double, 6, 7> lJacobian{
      { CalcJ11(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ12(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ13(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ14(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ15(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ16(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ17(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) },

      { CalcJ21(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ22(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ23(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ24(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ25(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ26(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ27(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) },

      { CalcJ31(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ32(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ33(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ34(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ35(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ36(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ37(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) },

      { CalcJ41(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ42(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ43(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ44(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ45(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ46(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ47(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) },

      { CalcJ51(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ52(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ53(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ54(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ55(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ56(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ57(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) },

      { CalcJ61(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ62(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ63(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ64(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ65(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ66(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ67(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) }
    };
    //clang-format on
    return lJacobian;
  }

} // namespace kinematics