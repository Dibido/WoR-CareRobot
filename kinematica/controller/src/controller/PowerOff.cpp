#include "controller/PowerOff.hpp"
#include "controller/Init.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <thread>

namespace controller
{
  PowerOff::PowerOff(){

  };

  PowerOff::~PowerOff(){};

  void PowerOff::entryAction(Context*)
  {
  }

  void PowerOff::doActivity(Context* aContext)
  {
    aContext->setState(std::make_shared<Init>());
  }

  void PowerOff::exitAction(Context*)
  {
  }
} // namespace controller