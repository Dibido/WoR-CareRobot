// Library
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>

// Local
#include "controller/ControllerConsts.hpp"
#include "controller/Move.hpp"
#include "controller/ReleaseCup.hpp"
#include "controller/WaitForCup.hpp"
#include "kinematics/Configuration.hpp"
#include "kinematics/EndEffector.hpp"
#include "robotcontroller/RobotControlPublisher.hpp"
namespace controller
{
  Move::Move(){

  };
  Move::~Move(){};

  void Move::entryAction(Context* aContext)
  {
    const double lSpeedFactor = 0.5;

    kinematics::EndEffector lEndEffector = kinematics::EndEffector(
        aContext->cup().object().position().x_m(),
        aContext->cup().object().position().y_m(),
        aContext->cup().object().position().z_m(), 0, M_PI_2, M_PI_2);

    kinematics::Configuration lConfiguration =
        aContext->configurationProvider()->inverseKinematics(
            lEndEffector, aContext->configuration());
    aContext->robotControl()->publish(0.5, lConfiguration);

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

    mArrivalTime =
        ros::Time::now() +
        ros::Duration(lMaxDeltaTheta / cJointSpeed_rads / lSpeedFactor);
    uint32_t time = mArrivalTime.toSec()-ros::Time::now().toSec() -1;
    ros::Duration lDuration(time);
    if (lDuration > ros::Duration(0))
    {
      lDuration.sleep();
    }
  }

  void Move::doActivity(Context* aContext)
  {
    if (ros::Time::now() >= mArrivalTime)
    {
      // aContext->setState(std::make_shared<ReleaseCup>());
      aContext->setState(std::make_shared<WaitForCup>());
    }
  }

  void Move::exitAction(Context* aContext)
  {
  }
} // namespace controller