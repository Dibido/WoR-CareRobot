#include "controller/Init.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/Ready.hpp"
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
  }

  void Init::doActivity(Context* aContext)
  {
    ROS_ERROR("DOING BEGIN1");
    aContext->goalConfiguration() = kinematics::Configuration();
    aContext->goalConfiguration().setTheta(3, -M_PI_2);
    aContext->goalConfiguration().setTheta(5, M_PI_2);
    aContext->goalConfiguration().setTheta(6, M_PI_4);
    aContext->robotControl()->publish(cSpeedFactor, aContext->goalConfiguration());
    aContext->setState(std::make_shared<Ready>());
  }

  void Init::exitAction(Context* aContext)
  {
    aContext->currentConfiguration() = aContext->goalConfiguration();
  }
} // namespace controller