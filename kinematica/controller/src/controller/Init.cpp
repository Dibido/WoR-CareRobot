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

  void Init::entryAction(Context* context)
  {
    context->configuration() = kinematics::Configuration();
    context->robotControl()->publish(cSpeedFactor, context->configuration());
  }

  void Init::doActivity(Context* context)
  {
    context->configuration() = kinematics::Configuration();
    context->robotControl()->publish(cSpeedFactor, context->configuration());
    context->setState(std::make_shared<Ready>());
  }

  void Init::exitAction(Context* context)
  {
  }
} // namespace controller