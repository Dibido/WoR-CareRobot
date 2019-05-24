// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/Init.hpp"
#include "controller/SoftStop.hpp"
namespace controller
{
  SoftStop::SoftStop(){

  };
  SoftStop::~SoftStop(){};

  void SoftStop::entryAction(Context* context)
  {
  }

  void SoftStop::doActivity(Context* context)
  {
    context->setState(std::make_shared<Init>());
  }

  void SoftStop::exitAction(Context* context)
  {
  }
} // namespace controller