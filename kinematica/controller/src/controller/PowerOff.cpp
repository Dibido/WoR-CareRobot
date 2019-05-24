#include "controller/PowerOff.hpp"
// Library
#include <chrono>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <thread>
namespace controller{
PowerOff::PowerOff(){

};

PowerOff::~PowerOff(){};

void PowerOff::entryAction(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;
}

void PowerOff::doActivity(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;
}

void PowerOff::exitAction(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;
}
}