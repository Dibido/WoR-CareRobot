#include "userinterface/GoalPublisher.hpp"

namespace userinterface
{
  GoalPublisher::GoalPublisher()
  {
    ros::NodeHandle goalPublisherNodeHandle;
    ros::Publisher chatter_pub =
        goalPublisherNodeHandle.advertise<geometry_msgs::Point>("goal", 1000);
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
    geometry_msgs::Point msg;
    msg.x = aPosition.x_m();
    msg.y = aPosition.y_m();
    msg.z = aPosition.z_m();

    // Send the message to the given topic.
    chatter_pub.publish(msg);
  }

} // namespace userinterface
