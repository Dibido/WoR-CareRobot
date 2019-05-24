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

  void Ready::entryAction(Context* aContext)
  {
    aContext->cup().object().width_m() = 0.0;
  }

  void Ready::doActivity(Context* aContext)
  {
    if (aContext->cup().object().width_m() > 0.0)
    {
      aContext->setState(std::make_shared<Move>());
    }
  }

  void Ready::exitAction(Context* aContext)
  {
  }
} // namespace controller