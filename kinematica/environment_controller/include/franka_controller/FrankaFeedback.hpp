#ifndef FRANKA_FEEDBACK_HPP
#define FRANKA_FEEDBACK_HPP
#include "environment_controller/EnvironmentController.hpp"
#include "robotcontroller_msgs/FrankaFeedback.h"
#include <ros/ros.h>
namespace franka_controller
{
  class FrankaFeedback
  {
      public:
    FrankaFeedback(
        const std::string& aTopicName,
        const std::shared_ptr<environment_controller::EnvironmentController>&
            aController);
    virtual ~FrankaFeedback() = default;

      private:
    void callback(const robotcontroller_msgs::FrankaFeedbackConstPtr& aMsg);
    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    std::shared_ptr<environment_controller::EnvironmentController>
        mEnvironmentController;
  };
} // namespace franka_controller

#endif