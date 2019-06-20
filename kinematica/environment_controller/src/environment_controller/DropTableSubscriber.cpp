#include "environment_controller/DropTableSubscriber.hpp"
#include "environment_controller/EnvironmentConsts.hpp"

namespace environment_controller
{

  DropTableSubscriber::DropTableSubscriber(
      const std::string& aTopicName,
      const std::shared_ptr<EnvironmentController>& aController)
      : mSubscriber(mHandle.subscribe(aTopicName,
                                      cQueue_size,
                                      &DropTableSubscriber::dropTableCallback,
                                      this)),
        mEnvironmentController(aController)
  {
  }

  void DropTableSubscriber::dropTableCallback(
      const kinematica_msgs::GoalConstPtr& aMsg)
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

  void DropTableSubscriber::selectGoalPosition(
      const Position& aPosition,
      __attribute__((unused)) bool aStaticGoal)
  {
    mEnvironmentController->provideDrop(aPosition);
  }
} // namespace environment_controller
