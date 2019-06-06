#ifndef CONTROLLER_CONSTS_HPP
#define CONTROLLER_CONSTS_HPP

#include <cmath>
#include <planning/Obstacle.hpp>
#include <stdint.h>
#include <string>

namespace controller
{
  const uint16_t cQueue_size = 1000;
  const std::string cRobotCommandTopicName = "robot_command";
  const std::string cRobotGripperTopicName = "robot_gripper";
  const std::string cRobotStopTopicName = "robot_stop";
  const double cGripperSpeed_ms = 0.1;
  const double cJoint1To4Speed_rads = 2.61799;
  const double cJoint5To7Speed_rads = M_PI;
  const double cSpeedFactor = 0.4;
  const double cJointSpeed_rads = M_PI / 180 * 150;
  const double cGripperWidth_m = 0.08;
  const double cWaitTime_s = 0.5;
  const uint64_t cS_to_nano_s = pow(10, 9);

  const planning::Obstacle cRobotObstacle{ 0.0f, 0.0f, 0.0f, 0.6f, 0.6f, 1.6f };
  const planning::Obstacle cFloorObstacle{
    0.0f, 0.0f, -0.05f, 2.0f, 2.0f, 0.1f
  };
} // namespace controller

#endif // CONTROLLER_CONSTS_HPP