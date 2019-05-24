// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/Ready.hpp"
#include "controller/ReleaseCup.hpp"

ReleaseCup::ReleaseCup(){

};
ReleaseCup::~ReleaseCup(){};

void ReleaseCup::entryAction(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;
}

void ReleaseCup::doActivity(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;
  context->setState(std::make_shared<Ready>());
}

void ReleaseCup::exitAction(Context* context)
{
  // std::cout << __PRETTY_FUNCTION__ << std::endl;
}