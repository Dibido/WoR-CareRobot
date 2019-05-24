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

  void Move::entryAction(Context* context)
  {
    const double lSpeedFactor = 0.5;

    kinematics::EndEffector lEndEffector = kinematics::EndEffector(
        context->cup().object().position().x_m(),
        context->cup().object().position().y_m(),
        context->cup().object().position().z_m(), 0, M_PI_2, M_PI_2);

    kinematics::Configuration lConfiguration =
        context->configurationProvider()->inverseKinematics(
            lEndEffector, context->configuration());
    context->robotControl()->publish(0.5, lConfiguration);

    double lMaxDeltaTheta = 0;
    for (size_t i = 0; i < lConfiguration.size; ++i)
    {
      if (lMaxDeltaTheta <
          std::abs(lConfiguration[i] - context->configuration()[i]))
      {
        lMaxDeltaTheta =
            std::abs(lConfiguration[i] - context->configuration()[i]);
      }
    }

    mArrivalTime =
        ros::Time::now() +
        ros::Duration(lMaxDeltaTheta / cJointSpeed_rads / lSpeedFactor);
  }

  void Move::doActivity(Context* context)
  {
    if (ros::Time::now() >= mArrivalTime)
    {
      // context->setState(std::make_shared<ReleaseCup>());
      context->setState(std::make_shared<WaitForCup>());
    }
  }

  void Move::exitAction(Context* context)
  {
  }
} // namespace controller