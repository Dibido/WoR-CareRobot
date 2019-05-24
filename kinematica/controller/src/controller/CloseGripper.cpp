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

  void CloseGripper::entryAction(Context* context)
  {
    context->gripperData() = robotcontroller::GripperData(
        context->cup().object().width_m(), cSpeedFactor);
    context->robotGripper()->moveGripper(context->gripperData());
    mGripperCloseTime =
        ros::Time::now() + ros::Duration(0.08 / 0.1 / cSpeedFactor);
  }

  void CloseGripper::doActivity(Context* context)
  {
    // std::cout << __PRETTY_FUNCTION__ << std::endl;
    // Verschil in width  / 0.1 is de tijd die het duurt voordat de gripper open
    // is
    if (ros::Time::now() >= mGripperCloseTime)
    {
      context->setState(std::make_shared<Ready>());
    }
  }

  void CloseGripper::exitAction(Context* context)
  {
    // std::cout << __PRETTY_FUNCTION__ << std::endl;
  }
} // namespace controller