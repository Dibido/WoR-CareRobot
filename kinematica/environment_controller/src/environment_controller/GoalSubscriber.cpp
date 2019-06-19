#include "environment_controller/GoalSubscriber.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

namespace environment_controller
{

  GoalSubscriber::GoalSubscriber(
      const std::string& aTopicName,
      const std::shared_ptr<EnvironmentController>& aController)
      : mSubscriber(mHandle.subscribe(aTopicName,
                                      cQueue_size,
                                      &GoalSubscriber::goalCallback,
                                      this)),
        mEnvironmentController(aController)
  {
  }

  void GoalSubscriber::goalCallback(const kinematica_msgs::GoalConstPtr& aMsg)
  {
    try
    {
      Position lPos(aMsg->position.x, aMsg->position.y, aMsg->position.z);
      selectGoalPosition(lPos);
    }
    catch (const std::exception& lE)
    {
      ROS_ERROR("%s", lE.what());
    }
  }

  void GoalSubscriber::selectGoalPosition(const Position& aPosition,
                                          bool)
  {
    mEnvironmentController->provideGoal(aPosition);
  }
} // namespace environment_controller