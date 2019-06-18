#ifndef FRANKA_CONSTS_HPP
#define FRANKA_CONSTS_HPP

#include <string>

namespace franka_controller
{
  const std::string cFrankaIp = "192.168.10.2";
  const unsigned short cQueueSize = 1000;
  const unsigned short cDegreesOfFreedom = 7;
  const std::string cTopicName = "/feedback/franka";
  const std::string cCartasian = "cartesian_reflex";
} // namespace franka_controller

#endif // FRANKA_CONSTS_HPP