#ifndef EMERGENCYSTOPSUBSCRIBER_PROVIDER_HPP
#define EMERGENCYSTOPSUBSCRIBER_PROVIDER_HPP

#include "environment_controller/EnvironmentController.hpp"
#include "environment_controller/IEmergencyStopProvider.hpp"
#include "kinematica_msgs/EmergencyStop.h"

namespace environment_controller
{

  /**
   * @brief  Class for the Goal position subscriber
   *
   */
  class EmergencyStopSubscriber : public IEmergencyStopProvider
  {

      public:
    /**
     * @brief Construct a new Goal Subscriber object
     *
     * @param aTopicName
     * @param aController
     */
    EmergencyStopSubscriber(
        const std::string& aTopicName,
        const std::shared_ptr<EnvironmentController>& aController);

    /**
     * @brief Destroy the Goal Subscriber object
     *
     */
    virtual ~EmergencyStopSubscriber() = default;

    /**
     * @brief Passes the emergency stop object to the EnvironmentController
     *
     * @param aStop
     */
    virtual void selectEmergencyStop(const bool aStop);

      private:
    /**
     * @brief Callback function for calling the emegergency stop. This
     * function will be called when the emergency stop is called on the
     * topic.
     *
     * @param aMsg A emergency stop msg
     */
    void emergencyStopCallback(
        const kinematica_msgs::EmergencyStopConstPtr& aMsg);

    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    std::shared_ptr<EnvironmentController> mEnvironmentController;
  };

} // namespace environment_controller

#endif // EMERGENCYSTOPSUBSCRIBER_PROVIDER_HPP