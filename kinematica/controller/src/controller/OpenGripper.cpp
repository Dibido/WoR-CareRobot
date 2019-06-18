#include "controller/OpenGripper.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/Init.hpp"
#include <iostream>
#include <ros/ros.h>
#include <thread>

namespace controller
{
  OpenGripper::OpenGripper(){

  };
  OpenGripper::~OpenGripper(){};

  void OpenGripper::entryAction(Context* aContext)
  {
    aContext->gripperData() =
        robotcontroller::GripperData(cGripperWidth_m, cSpeedFactor);
    aContext->robotGripper()->moveGripper(aContext->gripperData());
  }

  void OpenGripper::doActivity(Context* aContext)
  {
    std::unique_lock<std::mutex> lLock(aContext->feedbackMutex());
    aContext->feedbackDone().wait(lLock);
    transition(aContext);
  }

  void OpenGripper::exitAction(Context*)
  {
  }

  void OpenGripper::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<Init>());
  }

} // namespace controller