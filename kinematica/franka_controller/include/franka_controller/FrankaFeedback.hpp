#ifndef FRANKA_FEEDBACK_HPP
#define FRANKA_FEEDBACK_HPP

#include <ros/ros.h>
namespace franka_controller
{
  class FrankaFeedback
  {
      public:
    FrankaFeedback(const std::string& aTopicName);
    virtual ~FrankaFeedback() = default;
    void pubFeedback(bool aFeedback);

      private:
    ros::NodeHandle mNodeHandle;
    ros::Publisher mFrankaPub;
  };
} // namespace franka_controller
#endif