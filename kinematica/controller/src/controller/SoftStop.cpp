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

  void SoftStop::entryAction(Context*)
  {
  }

  void SoftStop::doActivity(Context* aContext)
  {
    aContext->setState(std::make_shared<Init>());
  }

  void SoftStop::exitAction(Context*)
  {
  }
} // namespace controller