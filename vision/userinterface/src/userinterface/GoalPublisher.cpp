#include "userinterface/GoalPublisher.hpp"

namespace userinterface
{
  GoalPublisher::GoalPublisher() : mMsgSent(false)
  {
    ros::NodeHandle goalPublisherNodeHandle;
    mChatter_pub =
        goalPublisherNodeHandle.advertise<kinematica_msgs::Goal>("goal", 1000);
  }

  GoalPublisher::~GoalPublisher()
  {
  }

  void GoalPublisher::selectGoalPosition(
      const environment_controller::Position& aPosition)
  {
    ros::spinOnce();
    // Create and fill message with aPosition.
    kinematica_msgs::Goal lMsg;

    lMsg.staticGoal = false;
    lMsg.position.x = aPosition.x_m();
    lMsg.position.y = aPosition.y_m();
    lMsg.position.z = aPosition.z_m();

    // Send the message to the given topic.
    mChatter_pub.publish(lMsg);
  }

} // namespace userinterface
