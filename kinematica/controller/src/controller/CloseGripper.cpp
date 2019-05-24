// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/CloseGripper.hpp"
#include "controller/Ready.hpp"

namespace controller
{
  CloseGripper::CloseGripper(){

  };
  CloseGripper::~CloseGripper(){};

  void CloseGripper::entryAction(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
    // Closegripper()
  }

  void CloseGripper::doActivity(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
    // Verschil in width  / 0.1 is de tijd die het duurt voordat de gripper open is
    // if(gripper.closed()){
    context->setState(std::make_shared<Ready>());
    // }
  }

  void CloseGripper::exitAction(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
  }
} // namespace controller