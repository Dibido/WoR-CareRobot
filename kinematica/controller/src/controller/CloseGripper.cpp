
#include "controller/CloseGripper.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/MoveToDropLocation.hpp"
#include "controller/Ready.hpp"
#include <iostream>
#include <ros/ros.h>
#include <thread>

namespace controller
{
  CloseGripper::CloseGripper(){

  };
  CloseGripper::~CloseGripper(){};

  void CloseGripper::entryAction(Context* aContext)
  {
    aContext->gripperData() = robotcontroller::GripperData(
        aContext->cup().object().width_m() - cGripperCorrection_m,
        cSpeedFactor);
    aContext->robotGripper()->moveGripper(aContext->gripperData());
  }

  void CloseGripper::doActivity(Context* aContext)
  {
    std::unique_lock<std::mutex> lLock(aContext->feedbackMutex());
    aContext->feedbackDone().wait(lLock);
    transition(aContext);
  }

  void CloseGripper::exitAction(Context*)
  {
  }

  void CloseGripper::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<MoveToDropLocation>());
  }
} // namespace controller