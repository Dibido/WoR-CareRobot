#include "controller/MoveToDropLocation.hpp"
#include "controller/ControllerConsts.hpp"
#include "controller/WaitForReleaseSignal.hpp"

#include <thread>
namespace controller
{
  MoveToDropLocation::MoveToDropLocation(){};

  void MoveToDropLocation::entryAction(Context* aContext)
  {
    kinematics::EndEffector lEndEffector = kinematics::EndEffector(
        -0.5, 0.5, aContext->cup().object().position().z_m(), 0, M_PI_2,
        M_PI_2);

    kinematics::Configuration lConfiguration =
        aContext->configurationProvider()->inverseKinematics(
            lEndEffector, aContext->configuration());
    aContext->robotControl()->publish(cSpeedFactor, lConfiguration);
    mArrivalTime = calculateArrivalTime(aContext, lConfiguration);
    ros::Duration lDuration(mArrivalTime.toSec() - ros::Time::now().toSec() -
                            cWaitTime_s);
    uint64_t lMovementDuration_ns = mArrivalTime.toNSec() -
                                    ros::Time::now().toNSec() -
                                    (uint64_t)(cWaitTime_s * pow(10, 9));
    if (lDuration > ros::Duration(0))
    {
      std::this_thread::sleep_for(
          std::chrono::nanoseconds(lMovementDuration_ns));
    }
  }

  void MoveToDropLocation::doActivity(Context* aContext)
  {
    if (ros::Time::now() >= mArrivalTime)
    {
      aContext->setState(std::make_shared<WaitForReleaseSignal>());
    }
  }

  void MoveToDropLocation::exitAction(Context* aContext)
  {
  }

  ros::Time MoveToDropLocation::calculateArrivalTime(
      Context* aContext,
      kinematics::Configuration lConfiguration)
  {
    double lMaxDeltaTheta = 0;
    for (size_t i = 0; i < lConfiguration.size; ++i)
    {
      if (lMaxDeltaTheta <
          std::abs(lConfiguration[i] - aContext->configuration()[i]))
      {
        lMaxDeltaTheta =
            std::abs(lConfiguration[i] - aContext->configuration()[i]);
      }
    }

    return ros::Time::now() +
           ros::Duration(lMaxDeltaTheta / cJointSpeed_rads / cSpeedFactor);
  }
} // namespace controller