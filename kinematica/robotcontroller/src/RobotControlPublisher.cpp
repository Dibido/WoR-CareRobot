#include "robotcontroller/RobotControlPublisher.hpp"

namespace robotcontroller
{

  RobotControlPublisher::RobotControlPublisher(ros::NodeHandle& lN,
                                               const std::string& lTopic,
                                               const uint16_t lQueue_size)
      : mN(lN),
        cTopic(lTopic),
        cQueue_size(lQueue_size),
        mRobotControl_pub(
            mN.advertise<robotcontroller_msgs::Control>(cTopic, cQueue_size))
  {
  }

  void RobotControlPublisher::publish(
      const double lSf,
      const kinematics::Configuration& lConfiguration)
  {
    robotcontroller_msgs::Control lMsg;

    lMsg.theta.resize(lConfiguration.size);
    std::copy(lConfiguration.getConfiguration().begin(),
              lConfiguration.getConfiguration().end(), lMsg.theta.begin());
    lMsg.sf = lSf;

    mRobotControl_pub.publish(lMsg);
  }

} // namespace robotcontroller