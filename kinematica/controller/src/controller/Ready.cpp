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
    context->cup().object().width_m() = 0.0;
  }

  void Ready::doActivity(Context* context)
  {
    if (context->cup().object().width_m() > 0.0)
    {
      context->setState(std::make_shared<Move>());
    }
  }

  void Ready::exitAction(Context* context)
  {
  }
} // namespace controller