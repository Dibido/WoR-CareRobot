#ifndef CONTROLLER_CONSTS_HPP
#define CONTROLLER_CONSTS_HPP

#include <cmath>
#include <stdint.h>
#include <string>
namespace controller
{
  const uint16_t cQueue_size = 1000;
  const std::string cRobotCommandTopicName = "robot_command";
  const std::string cRobotGripperTopicName = "robot_gripper";
  const std::string cRobotStopTopicName = "robot_stop";
  const double cGripperSpeed_ms = 0.1;
  const double cJointSpeed_rads = M_PI / 180 * 150;
} // namespace controller

#endif // CONTROLLER_CONSTS_HPP