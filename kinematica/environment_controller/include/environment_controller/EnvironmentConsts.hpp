#ifndef ENVIRONMENT_CONSTS_HPP
#define ENVIRONMENT_CONSTS_HPP

#include <cmath>
#include <stdint.h>
#include <string>
namespace environment_controller
{
  const uint16_t cQueue_size = 1000;
  const uint8_t cRate = 10;
  const std::string cObstacleTopicName = "/detectedObjects";
  const uint8_t cMaxRange_m = 100;
  const int8_t cMinRange_m = -100;
  const uint8_t cTooFast_ms = 10;
  const int8_t cTooSlow_ms = 0;
  const double cRobotX_m = 0.0;
  const double cRobotY_m = 0.0;
  const double cRobotZ_m = 0.0;
  const double cLow_rad = -M_PI;
  const double cHigh_rad = M_PI;
  const double cMinDistanceToRobotarm_m = 2.0;
} // namespace environment_controller

#endif // ENVIRONMENT_CONSTS_HPP