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
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
  }

  void WaitForCup::doActivity(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
    // If cup.timeOfArrival == ros::now(){
    context->setState(std::make_shared<CloseGripper>());
  }

  void WaitForCup::exitAction(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
  }
} // namespace controller