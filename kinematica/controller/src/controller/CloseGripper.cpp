// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/CloseGripper.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/Ready.hpp"

namespace controller
{
  CloseGripper::CloseGripper(){

  };
  CloseGripper::~CloseGripper(){};

  void CloseGripper::entryAction(Context* aContext)
  {
    aContext->gripperData() = robotcontroller::GripperData(
        aContext->cup().object().width_m(), cSpeedFactor);
    aContext->robotGripper()->moveGripper(aContext->gripperData());
    mGripperCloseTime =
        ros::Time::now() +
        ros::Duration(cGripperWidth_m / cGripperSpeed_ms / cSpeedFactor);
  }

  void CloseGripper::doActivity(Context* aContext)
  {
    if (ros::Time::now() >= mGripperCloseTime)
    {
      aContext->setState(std::make_shared<Ready>());
    }
  }

  void CloseGripper::exitAction(Context* aContext)
  {
  }
} // namespace controller