// Library
#include <iostream>
#include <ros/ros.h>
#include <thread>
// Local
#include "controller/ControllerConsts.hpp"
#include "controller/Ready.hpp"
#include "controller/ReleaseCup.hpp"

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
    uint32_t lMovementDuration_ns = mReleaseTime.toNSec() -
                                    ros::Time::now().toNSec() -
                                    cWaitTime_s * pow(10, 9);
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
      aContext->setState(std::make_shared<Ready>());
    }
  }

  void ReleaseCup::exitAction(Context*)
  {
  }
} // namespace controller