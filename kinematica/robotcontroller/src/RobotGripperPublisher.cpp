#include "robotcontroller/RobotGripperPublisher.hpp"

namespace robotcontroller
{

  RobotGripperPublisher::RobotGripperPublisher(ros::NodeHandle& lN,
                                               const std::string& lTopic,
                                               const uint16_t lQueue_size)
      : mN(lN),
        cTopic(lTopic),
        cQueue_size(lQueue_size),
        mRobotGripper_pub(
            mN.advertise<robotcontroller_msgs::Gripper>(cTopic, cQueue_size))
  {
  }

  void RobotGripperPublisher::moveGripper(
      const robotcontroller::GripperData& aGripperData)
  {
    robotcontroller_msgs::Gripper lMsg;

    lMsg.width = aGripperData.cWidth_m;
    lMsg.speedfactor = aGripperData.cSpeedfactor;
    lMsg.force = aGripperData.cForce_nm;
    lMsg.epsilon_inner = aGripperData.cEpsilonInner_m;
    lMsg.epsilon_outer = aGripperData.cEpsilonOuter_m;

    mRobotGripper_pub.publish(lMsg);
  }

} // namespace robotcontroller