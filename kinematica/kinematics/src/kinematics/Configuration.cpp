#include "kinematics/Configuration.hpp"
#include "kinematics/UtilityFunctions.hpp"
#include <ros/ros.h>

namespace kinematics
{
  Configuration::Configuration() : mResult(false)
  {
    mConfiguration.fill(0);
  }

  const double& Configuration::operator[](std::size_t aIndex) const
  {
    if (aIndex >= size)
    {
      throw std::invalid_argument(
          "Trying to access a non existing configuration joint: " +
          std::to_string(aIndex));
    }
    return mConfiguration[aIndex];
  }

  void Configuration::setTheta(std::size_t aIndex, double aTheta)
  {
    if (aIndex >= size)
    {
      throw std::invalid_argument(
          "Trying to set theta for a non existing theta: " +
          std::to_string(aIndex));
    }
    mConfiguration[aIndex] = constrainRadian(aTheta);
  }

  bool Configuration::result() const
  {
    return mResult;
  }

  void Configuration::setResult(bool aResult)
  {
    mResult = aResult;
  }

  const std::array<double, cKinematicsDoF>&
      Configuration::getConfiguration() const
  {
    return mConfiguration;
  }

} // namespace kinematics
