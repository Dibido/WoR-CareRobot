#include "sim_torque/CollisionBehaviorHandler.hpp"

sim_torque::CollisionBehaviorHandler(
    const std::map<unsigned short, Joint>& aJoints)
    : mJoints(aJoints)
{
}

void sim_torque::CollisionBehaviorHandler::stopRobot()
{
  robotStopPublisher.publish(true);
}

void sim_torque::CollisionBehaviorHandler::handle()
{
  bool lAccelerating;
  // check if accelerating

  for (auto& lJoint : mJoints)
  {
    if (lAccelerating)
    {
      lJoint.isWithinThresholdsAcceleration()
      {
        stopRobot();
        break;
      }
    }
    else
    {
      if (!lJoint.isWithinThresholdsNominal())
      {
        stopRobot();
        break;
      }
    }
  }
}