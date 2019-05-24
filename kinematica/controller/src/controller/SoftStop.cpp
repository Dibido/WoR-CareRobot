// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/SoftStop.hpp"
#include "controller/Init.hpp"
namespace controller
{
  SoftStop::SoftStop(){

  };
  SoftStop::~SoftStop(){};

  void SoftStop::entryAction(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
  }

  void SoftStop::doActivity(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
    context->setState(std::make_shared<Init>());
  }

  void SoftStop::exitAction(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
  }
} // namespace controller