
#include "kinematics/Link.hpp"
#include "kinematics/UtilityFunctions.hpp"
#include <rng/RandomNumberGenerator.hpp>
#include <ros/ros.h>
#include <stdexcept>

namespace kinematics
{
  Link::Link()
      : mA_m(0),
        mAlpha_rad(0),
        mD_m(0),
        mTheta_rad(0),
        mType(eJoint::STATIC),
        mMinValue(0),
        mMaxValue(0)
  {
  }

  Link::Link(double aA_m,
             double aAlpha_rad,
             double aConstant,
             eJoint aType,
             double aMinValue,
             double aMaxValue)
      : Link(aA_m, aAlpha_rad, 0, 0, aType, aMinValue, aMaxValue)
  {
    switch (mType)
    {
    case eJoint::PRISMATIC:
      mTheta_rad = aConstant;
      break;
    case eJoint::REVOLUTE:
      mD_m = aConstant;
      break;
    default:
      throw std::invalid_argument(
          "Passed Joint type is not valid for single "
          "Offset construction " +
          JointHelper::toString(aType));
      break;
    }
  }

  Link::Link(double aA_m,
             double aAlpha_rad,
             double aD_m,
             double aTheta_rad,
             eJoint aType,
             double aMinValue,
             double aMaxValue)
      : mA_m(aA_m),
        mAlpha_rad(aAlpha_rad),
        mD_m(aD_m),
        mTheta_rad(aTheta_rad),
        mType(aType),
        mMinValue(aMinValue),
        mMaxValue(aMaxValue)
  {
  }

  bool Link::isWithinConstraints(double aVariable) const
  {
    if (mType == eJoint::REVOLUTE)
    {
      aVariable = constrainRadian(aVariable);
    }
    return (mType == eJoint::STATIC ||
            (mMinValue < aVariable && aVariable < mMaxValue));
  }

  Matrix<double, 4, 4> Link::transformationMatrix(double aVariable) const
  {
    double aD_m = mD_m;
    double aTheta_rad = mTheta_rad;
    switch (mType)
    {
    case eJoint::PRISMATIC:
      aD_m += aVariable;
      break;
    case eJoint::REVOLUTE:
      aTheta_rad += aVariable;
      break;
    case eJoint::STATIC:
      // Do not change any of the values
      break;
    }
    return calculateTransformationMatrix(aD_m, aTheta_rad);
  }

  eJoint Link::getType() const
  {
    return mType;
  }

  double Link::generateRandomVariable() const
  {
    return rng::RandomNumberGenerator::GenerateInRange(mMinValue, mMaxValue);
  }

  Matrix<double, 4, 4>
      Link::calculateTransformationMatrix(double aD_m, double aTheta_rad) const
  {
    // clang-format off
  return Matrix<double, 4, 4>{
    { cos(aTheta_rad), -sin(aTheta_rad), 0, mA_m },
    { sin(aTheta_rad) * cos(mAlpha_rad), cos(aTheta_rad) * cos(mAlpha_rad), -sin(mAlpha_rad), -sin(mAlpha_rad) * aD_m },
    { sin(aTheta_rad) * sin(mAlpha_rad), cos(aTheta_rad) * sin(mAlpha_rad), cos(mAlpha_rad), cos(mAlpha_rad) * aD_m },
    { 0, 0, 0, 1 }
  };
    // clang-format on
  }
} // namespace kinematics
