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

  void WaitForCup::entryAction(Context* context)
  {
  }

  void WaitForCup::doActivity(Context* context)
  {
    if (context->cup().timeOfArrival() <= ros::Time::now())
    {
      context->setState(std::make_shared<CloseGripper>());
    }
  }

  void WaitForCup::exitAction(Context* context)
  {
  }
} // namespace controller