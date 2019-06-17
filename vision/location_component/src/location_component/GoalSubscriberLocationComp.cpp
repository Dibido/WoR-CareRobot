#include "location_component/GoalSubscriberLocationComp.hpp"

namespace location_component
{

  GoalSubscriberLocationComp::GoalSubscriberLocationComp(
      const std::string& aTopicName,
      const std::shared_ptr<location_component::DetectAGV>& aDetectAGV)
      : mSubscriber(mHandle.subscribe(aTopicName,
                                      cQueue_size,
                                      &GoalSubscriberLocationComp::goalCallback,
                                      this)),
        mDetectAGV(aDetectAGV)
  {
  }

  void GoalSubscriberLocationComp::goalCallback(
      const kinematica_msgs::GoalConstPtr& aMsg)
  {

    try
    {
      if (!aMsg->staticCup)
      {
        environment_controller::Position lPostition(
            aMsg->position.x, aMsg->position.y, aMsg->position.z);
        selectGoalPosition(lPostition);
      }
    }
    catch (const std::exception& lE)
    {
      ROS_ERROR("%s", lE.what());
    }
  }

  /* virtual*/ void GoalSubscriberLocationComp::selectGoalPosition(
      const environment_controller::Position& aPosition)
  {
    // Position isn't used bit needed for the interface
    aPosition.x_m();

    // Changing the boolean to let the location component know to look for an
    // AGV
    mDetectAGV->setDetectObject(true);
  }

} // namespace location_component