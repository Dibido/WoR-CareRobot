#include "controller/MoveToDropLocation.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/WaitForReleaseSignal.hpp"

#include <thread>
namespace controller
{
  MoveToDropLocation::MoveToDropLocation(){};

  void MoveToDropLocation::entryAction(Context* aContext)
  {
    kinematics::EndEffector lTargetLocation = kinematics::EndEffector(
        aContext->dropPosition().x_m(), aContext->dropPosition().y_m(),
        aContext->cup().object().height_m() + aContext->dropPosition().z_m(), 0,
        M_PI_2, M_PI_2);

    mTrajectoryProvider.createTrajectory(aContext, lTargetLocation,
                                         mTrajectory);
    mArrivalTime = ros::Time::now();
  }

  void MoveToDropLocation::doActivity(Context* aContext)
  {
    int64_t lMovementDuration_ns =
        mArrivalTime.toNSec() - ros::Time::now().toNSec();
    if (lMovementDuration_ns > 0)
    {
      std::this_thread::sleep_for(
          std::chrono::nanoseconds(lMovementDuration_ns));
    }
    if (mTrajectory.size() == 0)
    {
      aContext->setState(std::make_shared<WaitForReleaseSignal>());
      return;
    }
    else
    {
      kinematics::Configuration& lTargetConfiguration = mTrajectory.front();
      aContext->robotControl()->publish(cSpeedFactor, lTargetConfiguration);
      aContext->currentConfiguration() = aContext->goalConfiguration();
      mArrivalTime = mTrajectoryProvider.calculateArrivalTime(
          aContext, lTargetConfiguration);
      aContext->goalConfiguration() = lTargetConfiguration;
      ROS_DEBUG(
          "Move to \n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f\n- %.4f",
          lTargetConfiguration[0], lTargetConfiguration[1],
          lTargetConfiguration[2], lTargetConfiguration[3],
          lTargetConfiguration[4], lTargetConfiguration[5],
          lTargetConfiguration[6]);
      mTrajectory.pop();
    }

  } // namespace controller

  void MoveToDropLocation::exitAction(Context*)
  {
  }

} // namespace controller