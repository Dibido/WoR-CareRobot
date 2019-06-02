#include "userinterface/ReleaseTimePublisher.hpp"

namespace userinterface
{
  ReleaseTimePublisher::ReleaseTimePublisher()
  {
    ros::NodeHandle ReleaseTimePublisherNodeHandle;
    ros::Rate loop_rate(10);
  }

  ReleaseTimePublisher::~ReleaseTimePublisher()
  {
  }

  void ReleaseTimePublisher::selectReleaseTime(const uint8_t aReleaseTime_s)
  {
    ros::spinOnce();
    // Create and fill message.
    kinematica_msgs::ReleaseTime lMsg;
    lMsg.releaseTime = aReleaseTime_s;

    // Send message to given topic.
    chatter_pub.publish(lMsg);
  }

} // namespace userinterface
