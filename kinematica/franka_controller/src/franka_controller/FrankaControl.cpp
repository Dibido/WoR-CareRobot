#include "franka_controller/FrankaControl.hpp"
#include "MotionGenerator.hpp"
#include <ros/ros.h>

namespace franka_controller
{

  FrankaControl::FrankaControl(const std::string& anIp)
      : mRobot(anIp), mGripper(anIp), mStop(false), mFeedback(cTopicName)
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

  void FrankaControl::executeMovement(
      std::array<double, cDegreesOfFreedom>& aConfig,
      double aSpeedFactor)
  {
    try
    {
      MotionGenerator lMotionGenerator(aSpeedFactor, aConfig);
      mRobot.control(lMotionGenerator);
    }
    catch (const franka::Exception& lE)
    {
      std::string lString(lE.what());
      std::size_t lPos = lString.find(cCartasian);
      ROS_ERROR(lE.what());
      if (lPos == std::string::npos)
      {

        mRobot.automaticErrorRecovery();
        executeMovement(aConfig, aSpeedFactor);
      }
    }
    mFeedback.pubFeedback(true);
  }

  void FrankaControl::moveGripper(double aWidth, double aSpeedFactor)
  {
    try
    {
      mGripper.move(aWidth, aSpeedFactor);
    }
    catch (const franka::Exception& lE)
    {
      ROS_ERROR(lE.what());
    }
    mFeedback.pubFeedback(true);
  }

  void FrankaControl::stopRobot(bool aStop)
  {
    try
    {
      mStop = aStop;
      if (mStop)
        mRobot.stop();
    }
    catch (const franka::Exception& lE)
    {
      ROS_ERROR(lE.what());
    }
  }
} // namespace franka_controller