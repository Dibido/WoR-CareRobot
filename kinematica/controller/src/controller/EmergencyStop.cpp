
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

  void EmergencyStop::doActivity(Context*)
  {
  }

  void EmergencyStop::exitAction(Context* aContext)
  {
    aContext->robotStop()->publish(false);
  }
} // namespace controller