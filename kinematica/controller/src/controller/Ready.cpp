// Library
#include <iostream>
#include <ros/ros.h>

// Local
#include "controller/Move.hpp"
#include "controller/Ready.hpp"
namespace controller
{
  Ready::Ready(){};

  Ready::~Ready(){};

  void Ready::entryAction(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
  }

  void Ready::doActivity(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;

    // If location received
    context->setState(std::make_shared<Move>());
  }

  void Ready::exitAction(Context* context)
  {
    //std::cout << __PRETTY_FUNCTION__ << std::endl;
  }
} // namespace controller