#include "environment_controller/EmergencyStopSubscriber.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

namespace environment_controller
{

  EmergencyStopSubscriber::EmergencyStopSubscriber(
      const std::string& aTopicName,
      const std::shared_ptr<EnvironmentController>& aController)
      : mSubscriber(
            mHandle.subscribe(aTopicName,
                              cQueue_size,
                              &EmergencyStopSubscriber::emergencyStopCallback,
                              this)),
        mEnvironmentController(aController)
  {
  }

  void EmergencyStopSubscriber::emergencyStopCallback(
      const kinematica_msgs::EmergencyStopConstPtr& aMsg)
  {
    try
    {
      selectEmergencyStop(aMsg->stop);
    }
    catch (const std::exception& lE)
    {
      ROS_ERROR("%s", lE.what());
    }
  }

  void EmergencyStopSubscriber::selectEmergencyStop(const bool& aStop)
  {
    mEnvironmentController->executeHardstop(aStop);
  }
} // namespace environment_controller