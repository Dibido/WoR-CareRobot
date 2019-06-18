#include "franka_controller/FrankaFeedback.hpp"
#include "environment_controller/EnvironmentConsts.hpp"
namespace franka_controller
{
  FrankaFeedback::FrankaFeedback(
      const std::string& aTopicName,
      const std::shared_ptr<environment_controller::EnvironmentController>&
          aController)
      : mSubscriber(
            mHandle.subscribe(aTopicName,
                              environment_controller::cQueue_size,
                              &franka_controller::FrankaFeedback::callback,
                              this)),
        mEnvironmentController(aController)
  {
  }

  void FrankaFeedback::callback(
      const robotcontroller_msgs::FrankaFeedbackConstPtr& aMsg)
  {
    mEnvironmentController->frankaDoneMoving(aMsg->succes);
  }
} // namespace franka_controller