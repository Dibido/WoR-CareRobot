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

  void ReleaseCup::entryAction(Context* context)
  {
  }

  void ReleaseCup::doActivity(Context* context)
  {

    // Verschil in width  / 0.1 is de tijd die het duurt voordat de gripper open
    // is if (gripper.opened){
    context->setState(std::make_shared<Ready>());
  }

  void ReleaseCup::exitAction(Context* context)
  {
  }
} // namespace controller