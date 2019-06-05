#include "franka_controller/FrankaControl.hpp"
#include "MotionGenerator.hpp"
#include <ros/ros.h>

namespace franka_controller
{

  FrankaControl::FrankaControl(const std::string& anIp) : mRobot(anIp)
  {
    mRobot.setCollisionBehavior(
        { { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 } },
        { { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 } },
        { { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 } },
        { { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 } },
        { { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 } },
        { { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 } },
        { { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 } },
        { { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 } });
  }

  void FrankaControl::executeMovement(std::array<double, 7>& aConfig,
                                      double aSpeedFactor)
  {
    try
    {
      MotionGenerator lMotionGenerator(aSpeedFactor, aConfig);
      mRobot.control(lMotionGenerator);
    }
    catch (const franka::Exception& lE)
    {
      ROS_ERROR(lE.what());
    }
  }
} // namespace franka_controller