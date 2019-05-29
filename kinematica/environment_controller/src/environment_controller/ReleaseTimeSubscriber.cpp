#include "environment_controller/ReleaseTimeSubscriber.hpp"

namespace environment_controller
{

  ReleaseTimeSubscriber::ReleaseTimeSubscriber(
      const std::string& aTopicName,
      const std::shared_ptr<EnvironmentController>& aController)
      : mSubscriber(
            mHandle.subscribe(aTopicName,
                              cQueue_size,
                              &ReleaseTimeSubscriber::releaseTimeCallback,
                              this)),
        mEnvironmentController(aController)
  {
  }

  void ReleaseTimeSubscriber::releaseTimeCallback(
      const kinematica_msgs::ReleaseTimeConstPtr& aMsg)
  {
    try
    {
      uint8_t lReleaseTime(aMsg->releaseTime);
      selectReleaseTime(lReleaseTime);
    }
    catch (const std::exception& lE)
    {
      ROS_ERROR("%s", lE.what());
    }
  }

  void ReleaseTimeSubscriber::selectReleaseTime(const uint8_t aReleaseTime_s)
  {
    mEnvironmentController->provideReleaseTime(aReleaseTime_s);
  }
} // namespace environment_controller