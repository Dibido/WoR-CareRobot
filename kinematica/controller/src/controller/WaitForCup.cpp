// Library
#include "controller/ControllerConsts.hpp"
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <thread>

// Local
#include "controller/CloseGripperCup.hpp"
#include "controller/WaitForCup.hpp"
namespace controller
{
  WaitForCup::WaitForCup(){

  };
  WaitForCup::~WaitForCup(){};

  void WaitForCup::entryAction(Context* aContext)
  {
    ros::Duration lDuration =
        (aContext->cup().timeOfArrival() - ros::Time::now()) -
        ros::Duration(cWaitTime_s) -
        ros::Duration(cGripperWidth_m / cGripperSpeed_ms / cSpeedFactor);

    aContext->cup().timeOfArrival() -=
        ros::Duration(cGripperWidth_m / cGripperSpeed_ms / cSpeedFactor);

    uint64_t lMovementDuration_ns =
        lDuration.toNSec() - (uint64_t)(cWaitTime_s * cS_to_nano_s);
    if (lDuration > ros::Duration(0))
    {

      std::this_thread::sleep_for(
          std::chrono::nanoseconds(lMovementDuration_ns));
    }
  }

  void WaitForCup::doActivity(Context* aContext)
  {
    if (aContext->cup().timeOfArrival() <= ros::Time::now())
    {
      transition(aContext);
    }
  }

  void WaitForCup::exitAction(Context*)
  {
  }

  void WaitForCup::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<CloseGripperCup>());
  }
} // namespace controller
