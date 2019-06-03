/**
 * @file Sensor.hpp
 * @author Brandon Geldof
 * @brief the header file for the position data struct
 * @version 0.1
 * @date 2019-06-03
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <stdexcept>

#include "environment_controller/Position.hpp"
#include "environment_controller/Rotation.hpp"
#include "ros/ros.h"

namespace environment_controller
{
  /**
   * @brief The data struct for a position, the structs throws an exception when
   * the value is out of range
   * @see parameters for correct values
   * @author Gianni Monteban
   */
  struct Sensor
  {
      public:
    Sensor(const Position& aPosition,
           const Rotation& aRotation,
           const uint8_t aSensorID);
    /**
     * @brief Getter & setter
     *
     * @return Position&
     */
    Position& position();
    const Position& position() const;
    /**
     * @brief Getter & setter
     *
     * @return Rotation&
     */
    Rotation& rotation();
    const Rotation& rotation() const;
    /**
     * @brief Getter & setter
     *
     * @return uint8_t&
     */
    uint8_t& sensorID();
    const uint8_t& sensorID() const;

      private:
    Position mPosition;
    Rotation mRotation;
    uint8_t sensorID;
  };
} // namespace environment_controller

#endif // POSITION_HPP