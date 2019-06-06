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
    mRobot.setJointImpedance({ { 3000, 3000, 3000, 2500, 2500, 2000, 2000 } });
    mRobot.setCartesianImpedance({ { 3000, 3000, 3000, 300, 300, 300 } });
  }

  void FrankaControl::executeMovement(std::array<double, 7>& aConfig,
                                      double aSpeedFactor)
  {
    try
    {
      // aConfig[6] += 0.8;
      std::cout << aConfig[6] << std::endl;
      MotionGenerator lMotionGenerator(aSpeedFactor, aConfig);
      mRobot.control(lMotionGenerator);
    }
    catch (const franka::Exception& lE)
    {
      ROS_ERROR(lE.what());
    }
  }
} // namespace franka_controller