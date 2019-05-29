
#include "controller/EmergencyStop.hpp"
#include "controller/Init.hpp"
#include <iostream>
#include <ros/ros.h>

namespace controller
{
  EmergencyStop::EmergencyStop(){

  };
  EmergencyStop::~EmergencyStop(){};

  void EmergencyStop::entryAction(Context* aContext)
  {
    aContext->robotStop()->publish(true);
  }

  void EmergencyStop::doActivity(Context* aContext)
  {
    // aContext->setState(std::make_shared<Init>());
  }

  void EmergencyStop::exitAction(Context* aContext)
  {
    aContext->robotStop()->publish(false);
  }
} // namespace controller