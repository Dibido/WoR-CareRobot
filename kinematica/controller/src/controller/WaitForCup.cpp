// Library
#include "controller/ControllerConsts.hpp"
#include <iostream>
#include <ros/ros.h>
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
    if (lDuration > ros::Duration(0))
    {
      lDuration.sleep();
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