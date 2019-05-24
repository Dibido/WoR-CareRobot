// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/CloseGripper.hpp"
namespace controller
{
  CloseGripper::CloseGripper(){

  };
  CloseGripper::~CloseGripper(){};

  void CloseGripper::entryAction(Context* context)
  {
    // std::cout << __PRETTY_FUNCTION__ << std::endl;
  }

  void CloseGripper::doActivity(Context* context)
  {
    // std::cout << __PRETTY_FUNCTION__ << std::endl;
  }

  void CloseGripper::exitAction(Context* context)
  {
    // std::cout << __PRETTY_FUNCTION__ << std::endl;
  }
} // namespace controller