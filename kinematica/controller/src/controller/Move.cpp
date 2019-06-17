#include "controller/Move.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/Init.hpp"
#include "controller/OpenGripper.hpp"
#include "controller/WaitForCup.hpp"
#include "kinematics/EndEffector.hpp"
#include "robotcontroller/RobotControlPublisher.hpp"
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <thread>
namespace controller
{
  Move::Move(){};

  Move::~Move(){};

  void Move::entryAction(Context* aContext)
  {
    kinematics::EndEffector lTargetLocation =
        kinematics::EndEffector(aContext->cup().object().position().x_m(),
                                aContext->cup().object().position().y_m(),
                                aContext->cup().object().position().z_m() +
                                    aContext->cup().object().height_m() / 4,
                                0, M_PI_2, M_PI_2);
    mTrajectoryProvider.createTrajectory(aContext, lTargetLocation, mTrajectory,
                                         false, true);
    mArrivalTime = ros::Time::now();
  }

  void Move::doActivity(Context* aContext)
  {
    int64_t lMovementDuration_ns =
        mArrivalTime.toNSec() - ros::Time::now().toNSec();
    if (lMovementDuration_ns > 0)
    {
      std::this_thread::sleep_for(
          std::chrono::nanoseconds(lMovementDuration_ns));
    }
    if (mTrajectory.size() == 0)
    {
      transition(aContext);
      return;
    }
    else
    {
      kinematics::Configuration& lTargetConfiguration = mTrajectory.front();
      aContext->robotControl()->publish(cSpeedFactor, lTargetConfiguration);
      aContext->currentConfiguration() = aContext->goalConfiguration();
      mArrivalTime = mTrajectoryProvider.calculateArrivalTime(
          aContext, lTargetConfiguration);
      aContext->goalConfiguration() = lTargetConfiguration;
      ROS_DEBUG(
          "Move to \n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f",
          lTargetConfiguration[0], lTargetConfiguration[1],
          lTargetConfiguration[2], lTargetConfiguration[3],
          lTargetConfiguration[4], lTargetConfiguration[5],
          lTargetConfiguration[6]);
      mTrajectory.pop();
    }
  }

  void Move::exitAction(Context*)
  {
  }

  void Move::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<WaitForCup>());
  }
} // namespace controller
