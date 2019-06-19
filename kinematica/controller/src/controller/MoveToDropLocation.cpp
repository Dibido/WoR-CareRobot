#include "controller/MoveToDropLocation.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/WaitForReleaseSignal.hpp"

#include <thread>
namespace controller
{
  void MoveToDropLocation::entryAction(Context* aContext)
  {
    kinematics::EndEffector lTargetLocation = kinematics::EndEffector(
        aContext->dropPosition().x_m(), aContext->dropPosition().y_m(),
        aContext->cup().object().height_m() + aContext->dropPosition().z_m(), 0,
        M_PI_2, M_PI_2);
    mTrajectoryProvider.createTrajectory(aContext, lTargetLocation, mTrajectory,
                                         true, true);
    mArrivalTime = ros::Time::now();
  }
  void MoveToDropLocation::transition(Context* aContext)
  {
    aContext->setState(std::make_shared<WaitForReleaseSignal>());
  }
} // namespace controller