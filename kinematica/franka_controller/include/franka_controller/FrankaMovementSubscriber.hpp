#ifndef FRANKA_MOVEMENT_SUBSCRIBER_HPP
#define FRANKA_MOVEMENT_SUBSCRIBER_HPP

#include "FrankaControl.hpp"
#include "robotcontroller_msgs/Control.h"
#include <ros/ros.h>

namespace franka_controller
{
  class FrankaMovementSubscriber
  {

      public:
    /**
     * @brief Construct a new Franka Movement Subscriber object
     *
     * @param aTopicName
     * @param aFrankaControl
     */
    FrankaMovementSubscriber(
        const std::string& aTopicName,
        const std::shared_ptr<FrankaControl>& aFrankaControl);
    ~FrankaMovementSubscriber() = default;

    /**
     * @brief
     *
     * @param aMsg
     */
    void callBackMovement(const robotcontroller_msgs::ControlConstPtr& aMsg);

      private:
    ros::NodeHandle mNh;
    ros::Subscriber mMovementSubscriber;
    std::shared_ptr<FrankaControl> mFrankaControl;
  };
} // namespace franka_controller

#endif // FRANKA_MOVEMENT_SUBSCRIBER_HPP