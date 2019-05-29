// Library
#include "controller/ControllerConsts.hpp"
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <thread>

// Local
#include "controller/CloseGripper.hpp"
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
        ros::Duration(cWaitTime_s);
    uint32_t lMovementDuration_ns =
        lDuration.toNSec() - cWaitTime_s * pow(10, 9);
    if (lMovementDuration_ns > 0)
    {
      std::this_thread::sleep_for(
          std::chrono::nanoseconds(lMovementDuration_ns));
    }
  }

  void WaitForCup::doActivity(Context* aContext)
  {
    if (aContext->cup().timeOfArrival() <= ros::Time::now())
    {
      aContext->setState(std::make_shared<CloseGripper>());
    }
  }

  void WaitForCup::exitAction(Context*)
  {
  }
} // namespace controller