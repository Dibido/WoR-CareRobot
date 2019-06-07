#ifndef FRANKA_STOP_SUBSCRIBER_HPP
#define FRANKA_STOP_SUBSCRIBER_HPP

#include "FrankaControl.hpp"
#include "robotcontroller_msgs/Stop.h"
#include <ros/ros.h>
namespace franka_controller
{
  class FrankaStopSubscriber
  {

      public:
    /**
     * @brief Construct a new Franka Stop Subscriber object
     *
     * @param aTopicName
     * @param aFrankaControl
     */
    FrankaStopSubscriber(const std::string& aTopicName,
                         const std::shared_ptr<FrankaControl>& aFrankaControl);
    ~FrankaStopSubscriber() = default;

    /**
     * @brief
     *
     * @param aMsg
     */
    void stopCallBack(const robotcontroller_msgs::StopConstPtr& aMsg);

      private:
    ros::NodeHandle mNodeHandle;
    ros::Subscriber mFrankaStopSub;
    std::shared_ptr<FrankaControl> mFrankaControl;
  };

} // namespace franka_controller
#endif // FRANKA_STOP_SUBSCRIBER_HPP