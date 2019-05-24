// Library
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
  }

  void WaitForCup::doActivity(Context* aContext)
  {
    if (aContext->cup().timeOfArrival() <= ros::Time::now())
    {
      aContext->setState(std::make_shared<CloseGripper>());
    }
  }

  void WaitForCup::exitAction(Context* aContext)
  {
  }
} // namespace controller