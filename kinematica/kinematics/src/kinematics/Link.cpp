
#include "kinematics/Link.hpp"
#include <ros/ros.h>
#include <stdexcept>

namespace kinematics
{
  Link::Link(double a,
             double alpha,
             double constant,
             Joint type,
             double minValue,
             double maxValue)
      : Link(a, alpha, 0, 0, type, minValue, maxValue)
  {
    switch (type)
    {
    case Joint::PRISMATIC:
      theta = constant;
      break;
    case Joint::REVOLUTE:
      d = constant;
      break;
    default:
      throw std::invalid_argument(
          "Passed Joint type is not valid for single "
          "Offset construction " +
          JointHelper::toString(type));
      break;
    }
  }

  Link::Link(double a,
             double alpha,
             double d,
             double theta,
             Joint type,
             double minValue,
             double maxValue)
      : a(a),
        alpha(alpha),
        d(d),
        theta(theta),
        type(type),
        minValue(minValue),
        maxValue(maxValue)
  {
  }

  bool Link::isWithinConstraints(double variable) const
  {
    variable = constrainVariable(variable);
    return (type == Joint::STATIC ||
            (minValue < variable && variable < maxValue));
  }

  double Link::constrainVariable(double variable) const
  {
    if (type == Joint::REVOLUTE)
    {
      // Constrain value between -M_PI and M_PI if the Link is of type REVOLUTE
      variable = std::fmod(variable + M_PI, M_PI * 2);
      if (variable < 0)
      {
        variable += M_PI * 2;
      }
      variable -= M_PI;
    }
    
    if (minValue > variable)
    {
      variable = minValue;
    }
    else if (variable > maxValue)
    {
      variable = maxValue;
    }
    return variable;
  }

  Matrix<double, 4, 4> Link::transformationMatrix(double variable) const
  {
    if (isWithinConstraints(variable) == false)
    {
      ROS_WARN(
          "Link variable (%.4f) is out of allowed constraints (%.4f < v < "
          "%.4f)",
          variable, minValue, maxValue);
    }
    double newD = d;
    double newTheta = theta;
    switch (type)
    {
    case Joint::PRISMATIC:
      newD += variable;
      break;
    case Joint::REVOLUTE:
      newTheta += variable;
      break;
    case Joint::STATIC:
      // Do not change any of the values
      break;
    default:
      throw std::invalid_argument("Invalid Joint type: " +
                                  JointHelper::toString(type));
      break;
    }
    ROS_DEBUG("T: %s A: %.2f\talpha: %.2f\tD: %.2f\tTheta: %.2f",
              JointHelper::toString(type).c_str(), a, alpha, newD, newTheta);
    return calculateTransformationMatrix(newD, newTheta);
  }

  double Link::getA() const
  {
    return a;
  }
  double Link::getAlpha() const
  {
    return alpha;
  }
  double Link::getD() const
  {
    return d;
  }
  double Link::getTheta() const
  {
    return d;
  }
  Joint Link::getType() const
  {
    return type;
  }

  Matrix<double, 4, 4> Link::calculateTransformationMatrix(double d,
                                                           double theta) const
  {
    // clang-format off
    return Matrix<double, 4, 4>{
      { cos(theta)              , -sin(theta)              ,  0         ,  a              },
      { sin(theta) * cos(alpha) ,  cos(theta) * cos(alpha) , -sin(alpha), -sin(alpha) * d },
      { sin(theta) * sin(alpha) ,  cos(theta) * sin(alpha) ,  cos(alpha),  cos(alpha) * d },
      { 0                       ,  0                       ,  0         ,  1              }
    };
    // clang-format on
  }
} // namespace kinematics
