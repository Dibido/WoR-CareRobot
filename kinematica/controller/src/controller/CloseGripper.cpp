
#include "controller/CloseGripper.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/MoveToDropLocation.hpp"
#include "controller/Ready.hpp"
#include <iostream>
#include <ros/ros.h>
#include <thread>

namespace controller
{
  CloseGripper::CloseGripper(){

  };
  CloseGripper::~CloseGripper(){};

  void CloseGripper::entryAction(Context* aContext)
  {
    aContext->gripperData() = robotcontroller::GripperData(
        aContext->cup().object().width_m() - cGripperCorrection_m,
        cSpeedFactor);
    aContext->robotGripper()->moveGripper(aContext->gripperData());
    mGripperCloseTime =
        ros::Time::now() +
        ros::Duration(cGripperWidth_m / cGripperSpeed_ms / cSpeedFactor);

    ros::Duration lDuration(mGripperCloseTime.toSec() -
                            ros::Time::now().toSec() - cWaitTime_s);
    uint64_t lMovementDuration_ns = mGripperCloseTime.toNSec() -
                                    ros::Time::now().toNSec() -
                                    (uint64_t)(cWaitTime_s * cS_to_nano_s);
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
      transition(aContext);
    }
  }

  void CloseGripper::exitAction(Context*)
  {
  }

  void CloseGripper::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<MoveToDropLocation>());
  }
} // namespace controller