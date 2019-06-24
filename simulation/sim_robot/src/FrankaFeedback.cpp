#include "sim_robot/FrankaFeedback.hpp"
#include "robotcontroller_msgs/FrankaFeedback.h"
namespace franka_controller
{
  FrankaFeedback::FrankaFeedback(const std::string& aTopicName)
  {
    mFrankaPub = mNodeHandle.advertise<robotcontroller_msgs::FrankaFeedback>(
        aTopicName, 1000);
  }

  void FrankaFeedback::pubFeedback(bool aFeedback)
  {
    robotcontroller_msgs::FrankaFeedback lFeedback;
    lFeedback.succes = aFeedback;
    mFrankaPub.publish(lFeedback);
  }
} // namespace franka_controller