#include "controller/Move.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/Init.hpp"
#include "controller/ReleaseCup.hpp"
#include "controller/WaitForCup.hpp"
#include "kinematics/EndEffector.hpp"
#include "robotcontroller/RobotControlPublisher.hpp"
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>

namespace controller
{
  Move::Move(){};

  Move::~Move(){};

  void Move::entryAction(Context* aContext)
  {
    kinematics::EndEffector lTargetLocation = kinematics::EndEffector(
        aContext->cup().object().position().x_m(),
        aContext->cup().object().position().y_m(),
        aContext->cup().object().position().z_m(), 0, M_PI_2, M_PI_2);

    mTrajectoryProvider.createTrajectory(aContext, lTargetLocation,
                                         mTrajectory);
    mArrivalTime = ros::Time::now();
  }

  void Move::doActivity(Context* aContext)
  {
    if (mArrivalTime.sleepUntil(mArrivalTime))
    {
      if (mTrajectory.size() == 0)
      {
        aContext->setState(std::make_shared<WaitForCup>());
        return;
      }
      else
      {
        kinematics::Configuration& lTargetConfiguration = mTrajectory.front();
        aContext->robotControl()->publish(cSpeedFactor, lTargetConfiguration);
        mArrivalTime = mTrajectoryProvider.calculateArrivalTime(
            aContext,lTargetConfiguration);
        aContext->configuration() = lTargetConfiguration;
        ROS_DEBUG(
            "Move to \n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f",
            lTargetConfiguration[0], lTargetConfiguration[1],
            lTargetConfiguration[2], lTargetConfiguration[3],
            lTargetConfiguration[4], lTargetConfiguration[5],
            lTargetConfiguration[6]);
        mTrajectory.pop();
      }
    }
  }

  void Move::exitAction(Context*)
  {
    while (mTrajectory.empty() == false)
    {
      mTrajectory.pop();
    }
  }
} // namespace controller