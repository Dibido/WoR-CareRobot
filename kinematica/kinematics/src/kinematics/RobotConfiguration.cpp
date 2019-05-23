#include "kinematics/RobotConfiguration.hpp"
#include "kinematics/UtilityFunctions.hpp"
#include <ros/ros.h>

namespace kinematics
{
  RobotConfiguration::RobotConfiguration()
  {
    // Joint 0
    mRobotConfiguration[0] = (kinematics::Link(
        0, 0, 0.333, kinematics::eJoint::REVOLUTE, -2.8973, 2.8973));
    // Joint 1
    mRobotConfiguration[1] = (kinematics::Link(
        0, -M_PI_2, 0.0, kinematics::eJoint::REVOLUTE, -1.7628, 1.7628));
    // Joint 2
    mRobotConfiguration[2] = (kinematics::Link(
        0.0, M_PI_2, 0.316, kinematics::eJoint::REVOLUTE, -2.8973, 2.8973));
    // Joint 3
    mRobotConfiguration[3] = (kinematics::Link(
        0.0825, M_PI_2, 0.0, kinematics::eJoint::REVOLUTE, -3.0718, -0.0698));
    // Joint 4
    mRobotConfiguration[4] =
        (kinematics::Link(-0.0825, -M_PI_2, 0.384, kinematics::eJoint::REVOLUTE,
                          -2.8973, 2.8973));
    // Joint 5
    mRobotConfiguration[5] = (kinematics::Link(
        0.0, M_PI_2, 0.0, kinematics::eJoint::REVOLUTE, -0.0175, 3.7525));
    // Joint 6
    mRobotConfiguration[6] = (kinematics::Link(
        0.088, M_PI_2, 0, kinematics::eJoint::REVOLUTE, -2.8973, 2.8973));
    // flange
    mRobotConfiguration[7] =
        (kinematics::Link(0.0, 0.0, 0.107, 0.0, kinematics::eJoint::STATIC));
  }

  const Link& RobotConfiguration::operator[](std::size_t aIndex) const
  {
    if (aIndex >= size)
    {
      throw std::invalid_argument(
          "Trying to access a non existing configuration Link: " +
          std::to_string(aIndex));
    }
    return mRobotConfiguration[aIndex];
  }
} // namespace kinematics
