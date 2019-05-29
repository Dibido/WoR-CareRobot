// Library
#include <iostream>
#include <ros/ros.h>
#include <thread>
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

    ros::Duration lDuration(mGripperCloseTime.toSec() -
                            ros::Time::now().toSec() - cWaitTime_s);
    uint32_t lMovementDuration_ns = mGripperCloseTime.toNSec() -
                                    ros::Time::now().toNSec() -
                                    cWaitTime_s * pow(10, 9);
    if (lDuration > ros::Duration(0))
    {
      std::this_thread::sleep_for(
          std::chrono::nanoseconds(lMovementDuration_ns));
    }
  }

  void CloseGripper::doActivity(Context* aContext)
  {
    if (ros::Time::now() >= mGripperCloseTime)
    {
      aContext->setState(std::make_shared<Ready>());
    }
  }

  void CloseGripper::exitAction(Context*)
  {
  }
} // namespace controller