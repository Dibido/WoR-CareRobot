#include "controller/SoftStop.hpp"
#include "controller/Init.hpp"
#include <iostream>
#include <ros/ros.h>

namespace controller
{
  SoftStop::SoftStop(){

  };
  SoftStop::~SoftStop(){};

  void SoftStop::entryAction(Context* aContext)
  {
    aContext->robotStop()->publish(true);
  }

  void SoftStop::doActivity(Context* aContext)
  {
    transition(aContext);
  }

  void SoftStop::exitAction(Context* aContext)
  {
    aContext->robotStop()->publish(false);
  }

  void SoftStop::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<Init>());
  }
} // namespace controller