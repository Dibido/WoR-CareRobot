/**
 * @file Sensor.hpp
 * @author Brandon Geldof
 * @brief the header file for the sensor data struct
 * @version 0.1
 * @date 2019-06-03
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <stdexcept>

#include "environment_controller/Pose.hpp"
#include "ros/ros.h"

namespace environment_controller
{
  /**
   * @brief The data struct for the rotation of an object
   * @author Brandon Geldof
   */
  struct Sensor
  {
      public:
    /**
     * @brief Construct a new Sensor object
     *
     * @param aPose See Pose.hpp
     * @param aSensorID
     */
    Sensor(const uint8_t aSensorID, const Pose& aPose);
    /**
     * @brief Getter & setter
     *
     * @return Pose&
     */
    Pose& pose();
    const Pose& pose() const;
    /**
     * @brief Getter & setter
     *
     * @return uint8_t&
     */
    uint8_t sensorID();
    const uint8_t& sensorID() const;

      private:
    Pose mPose;
    uint8_t mSensorID;
  };
} // namespace environment_controller

#endif // POSITION_HPP