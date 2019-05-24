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
    // context->setState(std::make_shared<Init>());
  }

  void EmergencyStop::exitAction(Context* context)
  {
    context->robotStop()->publish(false);
  }
} // namespace controller