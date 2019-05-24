// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/EmergencyStop.hpp"
#include "controller/Init.hpp"
namespace controller
{
  EmergencyStop::EmergencyStop(){

  };
  EmergencyStop::~EmergencyStop(){};

  void EmergencyStop::entryAction(Context* context)
  {
    context->robotStop()->publish(true);
  }

  void EmergencyStop::doActivity(Context* context)
  {
    // std::cout << __PRETTY_FUNCTION__ << std::endl;
    // context->setState(std::make_shared<Init>());
  }

  void EmergencyStop::exitAction(Context* context)
  {
    context->robotStop()->publish(false);
    // std::cout << __PRETTY_FUNCTION__ << std::endl;
  }
} // namespace controller