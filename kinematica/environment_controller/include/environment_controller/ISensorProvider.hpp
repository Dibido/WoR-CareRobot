#ifndef ISENSOR_PROVIDER_HPP
#define ISENSOR_PROVIDER_HPP

#include "environment_controller/Sensor.hpp"
#include "ros/ros.h"

namespace environment_controller
{
  /**
   * @brief Interface class which can be used to register a sensor
   *
   * @pre A sensor is enabled and knows it's own pose
   * @post The sensor is registered in the environment controller
   */
  class ISensorProvider
  {
      public:
    /**
     * @brief Pure virtual function for passing the sensor pose and sensorID
     *
     * @param aSensor Data struct that consists of a Pose and a sensorID
     */
    virtual void provideSensor(const Sensor& aSensor) = 0;
  };
} // namespace environment_controller

#endif // ISENSOR_PROVIDER_HPP