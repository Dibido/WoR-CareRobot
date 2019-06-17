#ifndef LOCATION_COMPONENT_GOALSUBSCRIBER_PROVIDER_HPP
#define LOCATION_COMPONENT_GOALSUBSCRIBER_PROVIDER_HPP

#include "environment_controller/IGoalProvider.hpp"
#include "environment_controller/Position.hpp"
#include "kinematica_msgs/Goal.h"
#include "location_component/DetectAGV.hpp"
#include "ros/ros.h"

namespace location_component
{
  /**
   * @brief  Class for the Goal position subscriber
   *
   */
  class GoalSubscriberLocationComp : public environment_controller::IGoalProvider
  {

      public:
    /**
     * @brief Construct a new Goal Subscriber object
     *
     * @param aTopicName
     * @param aController
     */
    GoalSubscriberLocationComp(
        const std::string& aTopicName,
        const std::shared_ptr<location_component::DetectAGV>& aDetectAGV);

    /**
     * @brief Destroy the Goal Subscriber object
     *
     */
    virtual ~GoalSubscriberLocationComp() = default;

    virtual void
        selectGoalPosition(const environment_controller::Position& aPosition);

      private:
    /**
     * @brief Callback function for setting the x, y and z coordinates. This
     * function will be called when the position is published on the /goal
     * topic.
     *
     * @param aMsg A goal msg
     */
    void goalCallback(const kinematica_msgs::GoalConstPtr& aMsg);

    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    std::shared_ptr<location_component::DetectAGV> mDetectAGV;

    unsigned int cQueue_size = 1000;
  };

} // namespace location_component

#endif // GOALSUBSCRIBER_PROVIDER_HPP