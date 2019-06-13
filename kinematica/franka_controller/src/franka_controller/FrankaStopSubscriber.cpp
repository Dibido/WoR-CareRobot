#include "franka_controller/FrankaStopSubscriber.hpp"
#include "controller/ControllerConsts.hpp"

namespace franka_controller
{
  FrankaStopSubscriber::FrankaStopSubscriber(
      const std::string& aTopicName,
      const std::shared_ptr<FrankaControl>& aFrankaControl)
      : mFrankaStopSub(
            mNodeHandle.subscribe(aTopicName,
                                  controller::cQueue_size,
                                  &FrankaStopSubscriber::stopCallBack,
                                  this)),
        mFrankaControl(aFrankaControl)
  {
  }

  void FrankaStopSubscriber::stopCallBack(
      const robotcontroller_msgs::StopConstPtr& aMsg)
  {
    mFrankaControl->stopRobot(aMsg->stop);
  }
} // namespace franka_controller