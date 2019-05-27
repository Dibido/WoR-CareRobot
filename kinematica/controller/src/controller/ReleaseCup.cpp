// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/Ready.hpp"
#include "controller/ReleaseCup.hpp"
namespace controller
{
  ReleaseCup::ReleaseCup(){

  };
  ReleaseCup::~ReleaseCup(){};

  void ReleaseCup::entryAction(Context* aContext)
  {
  }

  void ReleaseCup::doActivity(Context* aContext)
  {
    aContext->setState(std::make_shared<Ready>());
  }

  void ReleaseCup::exitAction(Context* aContext)
  {
  }
} // namespace controller