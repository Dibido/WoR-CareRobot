#ifndef FRANKA_GRIPPER_SUBSCRIBER_HPP
#define FRANKA_GRIPPER_SUBSCRIBER_HPP

#include "FrankaControl.hpp"
#include "robotcontroller_msgs/Gripper.h"
#include <memory>
#include <ros/ros.h>
namespace franka_controller
{
  class FrankaGripperSubscriber
  {

      public:
    /**
     * @brief Construct a new Franka Gripper Subscriber object
     *
     * @param aTopicName
     * @param aFrankaControl
     */
    FrankaGripperSubscriber(
        const std::string& aTopicName,
        const std::shared_ptr<FrankaControl>& aFrankaControl);
    ~FrankaGripperSubscriber() = default;

    /**
     * @brief the callback for the gripper message
     *
     * @param aMsg
     */
    void gripperCallBack(const robotcontroller_msgs::GripperConstPtr& aMsg);

      private:
    ros::NodeHandle mNodeHandle;
    ros::Subscriber mFrankaGripperSub;
    std::shared_ptr<FrankaControl> mFrankaControl;
  };
} // namespace franka_controller

#endif // FRANKA_GRIPPER_SUBSCRIBER_HPP