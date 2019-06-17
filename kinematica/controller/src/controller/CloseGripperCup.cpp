
#include "controller/CloseGripperCup.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/MoveToDropLocation.hpp"
#include "controller/MoveToDropPatient.hpp"
#include "controller/Ready.hpp"
#include <iostream>
#include <ros/ros.h>
#include <thread>

namespace controller
{
  void CloseGripperCup::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<MoveToDropPatient>());
  }
} // namespace controller