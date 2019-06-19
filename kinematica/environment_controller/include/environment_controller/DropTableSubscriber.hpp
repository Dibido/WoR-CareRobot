#ifndef DROPTABLESUBSCRIBER_PROVIDER_HPP
#define DROPTABLESUBSCRIBER_PROVIDER_HPP

#include "environment_controller/EnvironmentController.hpp"
#include "environment_controller/IGoalProvider.hpp"
#include "kinematica_msgs/Goal.h"

namespace environment_controller
{

  /**
   * @brief  Class for the Goal position subscriber
   *
   */
  class DropTableSubscriber : public IGoalProvider
  {

      public:
    /**
     * @brief Construct a new Goal Subscriber object
     *
     * @param aTopicName
     * @param aController
     */
    DropTableSubscriber(
        const std::string& aTopicName,
        const std::shared_ptr<EnvironmentController>& aController);

    /**
     * @brief Destroy the Goal Subscriber object
     *
     */
    virtual ~DropTableSubscriber() = default;

    /**
     * @brief Passes the Goal position object to the EnvironmentController
     *
     * @param aPosition
     */
    virtual void selectGoalPosition(const Position& aPosition, bool staticGoal = true);

      private:
    /**
     * @brief Callback function for setting the x, y and z coordinates. This
     * function will be called when the position is published on the /goal
     * topic.
     *
     * @param aMsg A goal msg
     */
    void dropTableCallback(const kinematica_msgs::GoalConstPtr& aMsg);

    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    std::shared_ptr<EnvironmentController> mEnvironmentController;
  };

} // namespace environment_controller

#endif // DROPTABLESUBSCRIBER_PROVIDER_HPP