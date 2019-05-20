#include "robotcontroller/RobotStopPublisher.hpp"

namespace robotcontroller
{

  RobotStopPublisher::RobotStopPublisher(ros::NodeHandle& lN,
                                         const std::string& lTopic,
                                         const uint16_t lQueue_size)
      : mN(lN),
        cTopic(lTopic),
        cQueue_size(lQueue_size),
        mRobotControl_pub(
            mN.advertise<robotcontroller_msgs::Stop>(cTopic, cQueue_size))
  {
  }

  void RobotStopPublisher::publish(const bool lStop)
  {
    robotcontroller_msgs::Stop lMsg;

    lMsg.stop = lStop;

    mRobotControl_pub.publish(lMsg);
  }

} // namespace robotcontroller