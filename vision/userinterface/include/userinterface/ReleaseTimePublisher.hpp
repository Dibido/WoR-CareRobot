#ifndef RELEASE_TIME_PUBLISHER_HPP
#define RELEASE_TIME_PUBLISHER_HPP

#include "environment_controller/EnvironmentConsts.hpp"
#include "environment_controller/IReleaseTimeProvider.hpp"
#include "environment_controller/Position.hpp"
#include "kinematica_msgs/ReleaseTime.h"
#include "ros/ros.h"

namespace userinterface
{

  /**
   * @brief ReleaseTimePublisher class inherited from IReleaseTimeProvider.
   *
   * See
   * https://git.icaprojecten.nl/stash/projects/EBGURG/repos/wor-18-19-s2/browse/kinematica/environment_controller/include/environment_controller/IReleaseTimeProvider.hpp
   * for more information.
   *
   * @pre The desired amount of seconds for when a grabbed cup will be released,
   * is given by the user through the UI.
   * @post A ROS message with the desired duration will be sent to the provided
   * topic.
   */
  class ReleaseTimePublisher
      : public environment_controller::IReleaseTimeProvider
  {
      public:
    ReleaseTimePublisher();
    virtual ~ReleaseTimePublisher();

    /**
     * @brief Inherited method from base class.
     *
     * @param aReleaseTime_s
     */
    void selectReleaseTime(const uint8_t aReleaseTime_s) override;

    /**
     * @brief State boolean to store if the msg has been sent.
     */
    bool mMsgSent = false;

      private:
    ros::NodeHandle mReleaseTimePublisherNodeHandle;
    ros::Publisher mChatter_pub =
        mReleaseTimePublisherNodeHandle.advertise<kinematica_msgs::ReleaseTime>(
            environment_controller::cReleaseTimeTopicName,
            environment_controller::cQueue_size);
  };

} // namespace userinterface

#endif // RELEASE_TIME_PUBLISHER_HPP
