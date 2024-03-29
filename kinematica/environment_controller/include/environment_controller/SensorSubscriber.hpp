#ifndef SENSOR_SUBSCRIBER_HPP
#define SENSOR_SUBSCRIBER_HPP

#include "environment_controller/EnvironmentController.hpp"
#include "environment_controller/ISensorProvider.hpp"
#include "kinematica_msgs/Sensor.h"

namespace environment_controller
{
  /**
   * @brief Class the sensor topic subscriber
   *
   */
  class SensorSubscriber : public ISensorProvider
  {
      public:
    /**
     * @brief Construct a new Sensor Subscriber object
     *
     * @param aTopicName
     * @param aController Environment controller shared pointer
     */
    SensorSubscriber(const std::string& aTopicName,
                     const std::shared_ptr<EnvironmentController>& aController);

    /**
     * @brief Destroy the Sensor Subscriber object
     *
     */
    virtual ~SensorSubscriber() = default;

    /**
     * @brief Passes the Sensor data struct to the EnvironmentController
     *
     * @param aSensor Sensor struct object
     */
    virtual void provideSensor(const Sensor& aSensor);

      private:
    /**
     * @brief Callback function to construct a Pose object
     *
     * @param aMsg A Sensor msg
     */
    void sensorCallback(const kinematica_msgs::SensorConstPtr& aMsg);

    ros::NodeHandle mHandle;
    ros::Subscriber mSubscriber;
    std::shared_ptr<EnvironmentController> mEnvironmentController;
  };

} // namespace environment_controller

#endif // CUP_SUBSCRIBER_HPP