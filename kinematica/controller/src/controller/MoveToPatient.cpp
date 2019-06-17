#include "controller/MoveToPatient.hpp"
#include "controller/WaitForReleaseSignal.hpp"

namespace controller
{
  void MoveToPatient::entryAction(Context* aContext)
  {
    kinematics::EndEffector lTargetLocation = kinematics::EndEffector(
        aContext->patientPosition().x_m(), aContext->patientPosition().y_m(),
        aContext->patientPosition().z_m(), 0, M_PI_2, M_PI_2);

    mTrajectoryProvider.createTrajectory(aContext, lTargetLocation, mTrajectory,
                                         false, false);
    mArrivalTime = ros::Time::now();
  }
  void MoveToPatient::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<WaitForReleaseSignal>());
  }
} // namespace controller