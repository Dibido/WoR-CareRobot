#ifndef RELEASE_TIME_SUBSCRIBER_HPP
#define RELEASE_TIME_SUBSCRIBER_HPP

#include "environment_controller/EnvironmentController.hpp"
#include "environment_controller/IReleaseTimeProvider.hpp"
#include "kinematica_msgs/ReleaseTime.h"

namespace environment_controller
{

  /**
   * @brief Class for the ReleaseTime subscriber
   *
   */
  class ReleaseTimeSubscriber : public IReleaseTimeProvider
  {

      public:
    /**
     * @brief Construct a new Release Time Subscriber object
     *
     * @param aTopicName
     * @param aController
     */
    ReleaseTimeSubscriber(
        const std::string& aTopicName,
        const std::shared_ptr<EnvironmentController>& aController);

    /**
     * @brief Destroy the Release Time Subscriber object
     *
     */
    virtual ~ReleaseTimeSubscriber() = default;

    /**
     * @brief Callback function for setting the ReleaseTime. This function will
     * be called when the release time is published on the /release topic.
     *
     * @param aMsg Release Time message
     */
    void releaseTimeCallback(const kinematica_msgs::ReleaseTimeConstPtr& aMsg);

    /**
     * @brief Passes the Release Time (in seconds) to the EnvironmentController
     *
     * @param aReleaseTime_s Release time in seconds
     */
    virtual void selectReleaseTime(const uint8_t aReleaseTime_s);

      private:
    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    std::shared_ptr<EnvironmentController> mEnvironmentController;
  };

} // namespace environment_controller

#endif // RELEASE_TIME_SUBSCRIBER_HPP