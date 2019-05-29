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
    aContext->cup() = environment_controller::Cup(
        environment_controller::Object(
            environment_controller::Position(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.0,
            0.0, ros::Time(0), 0),
        ros::Time(0));
    aContext->dropPosition() = environment_controller::Position(0.0, 0.0, 0.0);
    aContext->releaseTime_s() = -1;
  }

  void Ready::doActivity(Context* aContext)
  {
    if (aContext->cup().object().width_m() > 0.0)
    {
      aContext->setState(std::make_shared<Move>());
    }
  }

  void Ready::exitAction(Context*)
  {
  }
} // namespace controller