#ifndef SENSOR_PUBLISHER_HPP
#define SENSOR_PUBLISHER_HPP

#include "environment_controller/EnvironmentController.hpp"
#include "environment_controller/ISensorProvider.hpp"
#include "kinematica_msgs/Sensor.h"

namespace lidar_application
{
  /**
   * @brief Class the sensor topic subscriber
   *
   */
  class SensorPublisher : public environment_controller::ISensorProvider
  {
      public:
    /**
     * @brief Construct a new Sensor Publisher object
     *
     * @param aTopicName
     * @param aController Environment controller shared pointer
     */
    SensorPublisher(ros::NodeHandle& aHandle,
                    const std::string& aTopic,
                    const uint16_t aQueue_size);

    /**
     * @brief Destroy the Sensor Subscriber object
     *
     */
    virtual ~SensorPublisher() = default;

    /**
     * @brief Passes the Sensor data struct to the EnvironmentController
     *
     * @param aSensor Sensor struct object
     */
    virtual void provideSensor(const environment_controller::Sensor& aSensor);

      private:
    ros::NodeHandle& mHandle;
    const std::string& cTopic;
    const uint16_t cQueue_size;
    ros::Publisher mSensor_pub;
  };

} // namespace lidar_application

#endif // SENSOR_PUBLISHER_HPP