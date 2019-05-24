// Local
#include "controller/Context.hpp"
#include "controller/Init.hpp"
#include "controller/PowerOff.hpp"

// Libary
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <thread>

Context::Context()
{
}

void Context::setState(const std::shared_ptr<State>& state)
{
  //std::cout << __PRETTY_FUNCTION__ << std::endl;

  if (currentState)
  {
    currentState->exitAction(*this);
  }
  currentState = state;

  currentState->entryAction(*this);
}

void Context::run()
{
  if (!currentState)
  {
    setState(std::make_shared<Init>());
  }
  int i = 0;
  while (ros::ok() && typeid(*currentState) != typeid(PowerOff))
  {
      currentState->doActivity(*this);
      if(++i > 10000){
        setState(std::make_shared<PowerOff>());
      }
  }
}