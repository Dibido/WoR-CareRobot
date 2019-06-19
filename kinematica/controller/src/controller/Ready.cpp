#include "controller/Ready.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/Move.hpp"
#include "controller/MoveToPatient.hpp"
#include <iostream>
#include <ros/ros.h>

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
    aContext->gripperData() =
        robotcontroller::GripperData(cGripperWidth_m, cSpeedFactor);
    aContext->robotGripper()->moveGripper(aContext->gripperData());
    aContext->patientPosition() =
        environment_controller::Position(0.0, 0.0, 0.0);
  }

  void Ready::doActivity(Context* aContext)
  {
    if (aContext->cup().object().width_m() > 0.0)
    {
      aContext->setState(std::make_shared<Move>());
    }
    else if (aContext->patientPosition().x_m() != 0.0 ||
             aContext->patientPosition().y_m() != 0.0 ||
             aContext->patientPosition().z_m() != 0.0)
    {
      aContext->setState(std::make_shared<MoveToPatient>());
    }
  }

  void Ready::exitAction(Context*)
  {
  }

  void Ready::transition(Context*)
  {
  }
} // namespace controller