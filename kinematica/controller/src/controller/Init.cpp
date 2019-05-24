#include "controller/Init.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/Ready.hpp"
// Library
#include <chrono>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <thread>
namespace controller
{
  Init::Init(){

  };

  Init::~Init(){};

  void Init::entryAction(Context* aContext)
  {
    aContext->configuration() = kinematics::Configuration();
    aContext->robotControl()->publish(cSpeedFactor, aContext->configuration());
  }

  void Init::doActivity(Context* aContext)
  {
    aContext->setState(std::make_shared<Ready>());
  }

  void Init::exitAction(Context* aContext)
  {
  }
} // namespace controller