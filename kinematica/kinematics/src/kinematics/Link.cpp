
#include "kinematics/Link.hpp"
#include <ros/ros.h>
#include <stdexcept>

namespace kinematics
{
  Link::Link(double lA_m,
             double lAlpha_rad,
             double lConstant,
             eJoint lType,
             double lMinValue,
             double lMaxValue)
      : Link(lA_m, lAlpha_rad, 0, 0, lType, lMinValue, lMaxValue)
  {
    switch (mType)
    {
    case eJoint::PRISMATIC:
      mTheta_rad = lConstant;
      break;
    case eJoint::REVOLUTE:
      mD_m = lConstant;
      break;
    default:
      throw std::invalid_argument(
          "Passed Joint type is not valid for single "
          "Offset construction " +
          JointHelper::toString(lType));
      break;
    }
  }

  Link::Link(double lA_m,
             double lAlpha_rad,
             double lD_m,
             double lTheta_rad,
             eJoint lType,
             double lMinValue,
             double lMaxValue)
      : mA_m(lA_m),
        mAlpha_rad(lAlpha_rad),
        mD_m(lD_m),
        mTheta_rad(lTheta_rad),
        mType(lType),
        mMinValue(lMinValue),
        mMaxValue(lMaxValue)
  {
  }

  bool Link::isWithinConstraints(double lVariable) const
  {
    if (mType == eJoint::REVOLUTE)
    {
      lVariable = constrainVariable(lVariable);
    }
    return (mType == eJoint::STATIC ||
            (mMinValue < lVariable && lVariable < mMaxValue));
  }

  double Link::constrainVariable(double lVariable) const
  {
    if (mType == eJoint::REVOLUTE)
    {
      // Constrain value between -M_PI and M_PI if the Link is of type REVOLUTE
      lVariable = std::fmod(lVariable + M_PI, M_PI * 2);
      if (lVariable < 0)
      {
        lVariable += M_PI * 2;
      }
      lVariable -= M_PI;
    }

    if (mMinValue > lVariable)
    {
      lVariable = mMinValue;
    }
    else if (lVariable > mMaxValue)
    {
      lVariable = mMaxValue;
    }
    return lVariable;
  }

  Matrix<double, 4, 4> Link::transformationMatrix(double lVariable) const
  {
    if (isWithinConstraints(lVariable) == false)
    {
      ROS_WARN(
          "Link variable (%.4f) is out of allowed constraints (%.4f < v < "
          "%.4f)",
          lVariable, mMinValue, mMaxValue);
    }
    double lD_m = mD_m;
    double lTheta_rad = mTheta_rad;
    switch (mType)
    {
    case eJoint::PRISMATIC:
      lD_m += lVariable;
      break;
    case eJoint::REVOLUTE:
      lTheta_rad += lVariable;
      break;
    case eJoint::STATIC:
      // Do not change any of the values
      break;
    default:
      throw std::invalid_argument("Invalid Joint type: " +
                                  JointHelper::toString(mType));
      break;
    }
    ROS_DEBUG("T: %s A: %.2f\talpha: %.2f\tD: %.2f\tTheta: %.2f",
              JointHelper::toString(mType).c_str(), mA_m, mAlpha_rad, lD_m,
              lTheta_rad);
    return calculateTransformationMatrix(lD_m, lTheta_rad);
  }

  double Link::getA() const
  {
    return mA_m;
  }
  double Link::getAlpha() const
  {
    return mAlpha_rad;
  }
  double Link::getD() const
  {
    return mD_m;
  }
  double Link::getTheta() const
  {
    return mTheta_rad;
  }
  eJoint Link::getType() const
  {
    return mType;
  }

  Matrix<double, 4, 4>
      Link::calculateTransformationMatrix(double lD_m, double lTheta_rad) const
  {
    // clang-format off
  return Matrix<double, 4, 4>{
    { cos(lTheta_rad), -sin(lTheta_rad), 0, mA_m },
    { sin(lTheta_rad) * cos(mAlpha_rad), cos(lTheta_rad) * cos(mAlpha_rad), -sin(mAlpha_rad), -sin(mAlpha_rad) * lD_m },
    { sin(lTheta_rad) * sin(mAlpha_rad), cos(lTheta_rad) * sin(mAlpha_rad), cos(mAlpha_rad), cos(mAlpha_rad) * lD_m },
    { 0, 0, 0, 1 }
  };
    // clang-format on
  }
} // namespace kinematics
