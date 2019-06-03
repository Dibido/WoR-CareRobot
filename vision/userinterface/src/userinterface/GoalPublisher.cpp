#include "userinterface/GoalPublisher.hpp"

namespace userinterface
{
  GoalPublisher::GoalPublisher()
  {
    ros::NodeHandle goalPublisherNodeHandle;
    ros::Publisher chatter_pub =
        goalPublisherNodeHandle.advertise<kinematica_msgs::Goal>("goal", 1000);
    ros::Rate loop_rate(10);
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
    lMsg.position.x = aPosition.x_m();
    lMsg.position.y = aPosition.y_m();
    lMsg.position.z = aPosition.z_m();

    // Send the message to the given topic.
    chatter_pub.publish(lMsg);
  }

} // namespace userinterface
