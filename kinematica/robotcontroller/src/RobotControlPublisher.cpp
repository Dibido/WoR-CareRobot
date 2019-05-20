#include "robotcontroller/RobotControlPublisher.hpp"

namespace robotcontroller
{

  RobotControlPublisher::RobotControlPublisher(ros::NodeHandle& lN,
                                               const std::string& lTopic,
                                               const uint16_t lQue_size)
      : mN(lN),
        cTopic(lTopic),
        cQue_size(lQue_size),
        mRobotControl_pub(
            mN.advertise<robotcontroller_msgs::Control>(cTopic, cQue_size))
  {
  }

  void RobotControlPublisher::publish(const double lSf,
                                      const std::vector<double>& lConfiguration)
  {
    robotcontroller_msgs::Control lMsg;

    lMsg.theta = lConfiguration;
    lMsg.sf = lSf;

    mRobotControl_pub.publish(lMsg);
  }

} // namespace robotcontroller