#include "robotcontroller/RobotStopPublisher.hpp"

namespace robotcontroller
{

  RobotStopPublisher::RobotStopPublisher(ros::NodeHandle& lN,
                                         const std::string& lTopic,
                                         const uint16_t lQue_size)
      : mN(lN),
        cTopic(lTopic),
        cQue_size(lQue_size),
        mRobotControl_pub(
            mN.advertise<robotcontroller_msgs::Stop>(cTopic, cQue_size))
  {
  }

  void RobotStopPublisher::publish(const bool lStop)
  {
    robotcontroller_msgs::Stop lMsg;

    lMsg.stop = lStop;

    mRobotControl_pub.publish(lMsg);
  }

} // namespace robotcontroller