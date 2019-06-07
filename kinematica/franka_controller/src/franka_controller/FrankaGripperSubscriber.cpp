#include "franka_controller/FrankaGripperSubscriber.hpp"
#include "controller/ControllerConsts.hpp"
namespace franka_controller
{
  FrankaGripperSubscriber::FrankaGripperSubscriber(
      const std::string& aTopicName,
      const std::shared_ptr<FrankaControl>& aFrankaControl)
      : mFrankaGripperSub(
            mNodeHandle.subscribe(aTopicName,
                                  controller::cQueue_size,
                                  &FrankaGripperSubscriber::gripperCallBack,
                                  this)),
        mFrankaControl(aFrankaControl)
  {
  }

  void FrankaGripperSubscriber::gripperCallBack(
      const robotcontroller_msgs::GripperConstPtr& aMsg)
  {
    double lWidth = aMsg->width;
    double lSpeedFactor = aMsg->speedfactor;
    mFrankaControl->moveGripper(lWidth, lSpeedFactor);
  }
} // namespace franka_controller
