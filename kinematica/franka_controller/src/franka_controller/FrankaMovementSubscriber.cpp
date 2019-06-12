#include "franka_controller/FrankaMovementSubscriber.hpp"
#include "franka_controller/FrankaConsts.hpp"
namespace franka_controller
{
  FrankaMovementSubscriber::FrankaMovementSubscriber(
      const std::string& aTopicName,
      const std::shared_ptr<FrankaControl>& aFrankaControl)
      : mMovementSubscriber(
            mNh.subscribe(aTopicName,
                          cQueueSize,
                          &FrankaMovementSubscriber::callBackMovement,
                          this)),
        mFrankaControl(aFrankaControl)
  {
  }
  void FrankaMovementSubscriber::callBackMovement(
      const robotcontroller_msgs::ControlConstPtr& aMsg)
  {
    std::vector<double> lMoves = aMsg->theta;
    double lSpeedFactor = aMsg->sf;
    std::array<double, cDegreesOfFreedom> lArrayMoves;
    std::copy_n(lMoves.begin(), cDegreesOfFreedom, lArrayMoves.begin());
    mFrankaControl->executeMovement(lArrayMoves, lSpeedFactor);
  }
} // namespace franka_controller