// Library
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <thread>

// Local
#include "controller/ControllerConsts.hpp"
#include "controller/Move.hpp"
#include "controller/ReleaseCup.hpp"
#include "controller/WaitForCup.hpp"
#include "kinematics/EndEffector.hpp"
#include "robotcontroller/RobotControlPublisher.hpp"
namespace controller
{
  Move::Move(){

  };
  Move::~Move(){};

  void Move::entryAction(Context* aContext)
  {
    kinematics::EndEffector lEndEffector = kinematics::EndEffector(
        aContext->cup().object().position().x_m(),
        aContext->cup().object().position().y_m(),
        aContext->cup().object().position().z_m(), 0, M_PI_2, M_PI_2);

    kinematics::Configuration lConfiguration =
        aContext->configurationProvider()->inverseKinematics(
            lEndEffector, aContext->configuration());
    aContext->robotControl()->publish(cSpeedFactor, lConfiguration);
    mArrivalTime = Move::calculateArrivalTime(aContext, lConfiguration);
    ros::Duration lDuration(mArrivalTime.toSec() - ros::Time::now().toSec() -
                            cWaitTime_s);
    uint32_t lMovementDuration_ns = mArrivalTime.toNSec() -
                                    ros::Time::now().toNSec() -
                                    cWaitTime_s * pow(10, 9);
    if (lDuration > ros::Duration(0))
    {
      std::this_thread::sleep_for(
          std::chrono::nanoseconds(lMovementDuration_ns));
    }
  }

  void Move::doActivity(Context* aContext)
  {
    if (ros::Time::now() >= mArrivalTime)
    {
      aContext->setState(std::make_shared<WaitForCup>());
    }
  }

  void Move::exitAction(Context*)
  {
  }

  ros::Time Move::calculateArrivalTime(Context* aContext,
                                       kinematics::Configuration lConfiguration)
  {
    double lMaxDeltaTheta = 0;
    for (size_t i = 0; i < lConfiguration.size; ++i)
    {
      if (lMaxDeltaTheta <
          std::abs(lConfiguration[i] - aContext->configuration()[i]))
      {
        lMaxDeltaTheta =
            std::abs(lConfiguration[i] - aContext->configuration()[i]);
      }
    }

    return ros::Time::now() +
           ros::Duration(lMaxDeltaTheta / cJointSpeed_rads / cSpeedFactor);
  }

} // namespace controller