#include "controller/ReleaseCup.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/Init.hpp"
#include <iostream>
#include <ros/ros.h>
#include <thread>

namespace controller
{
  ReleaseCup::ReleaseCup(){

  };
  ReleaseCup::~ReleaseCup(){};

  void ReleaseCup::entryAction(Context* aContext)
  {
    aContext->gripperData() =
        robotcontroller::GripperData(cGripperWidth_m, cSpeedFactor);
    aContext->robotGripper()->moveGripper(aContext->gripperData());
    mReleaseTime =
        ros::Time::now() +
        ros::Duration(cGripperWidth_m / cGripperSpeed_ms / cSpeedFactor);
    ros::Duration lDuration(mReleaseTime.toSec() - ros::Time::now().toSec() -
                            cWaitTime_s);
    uint64_t lMovementDuration_ns = mReleaseTime.toNSec() -
                                    ros::Time::now().toNSec() -
                                    (uint64_t)(cWaitTime_s * cS_to_nano_s);
    if (lDuration > ros::Duration(0))
    {
      std::this_thread::sleep_for(
          std::chrono::nanoseconds(lMovementDuration_ns));
    }
  }

  void ReleaseCup::doActivity(Context* aContext)
  {
    if (ros::Time::now() >= mReleaseTime)
    {
      std::this_thread::sleep_for(
          std::chrono::seconds(( uint64_t )cSafeWaitTime_s));
      aContext->setState(std::make_shared<Init>());
    }
  }

  void ReleaseCup::exitAction(Context*)
  {
  }
} // namespace controller