#ifndef OBJECT_HPP
#define OBJECT_HPP

#include "Position.hpp"
#include "ros/time.h"

namespace environment_controller
{
  /**
   * @brief data structure object, if a parameter is not between the correct
   * values it will throw an exception
   * @see parameters for correct interface data
   * @author Gianni Monteban
   */
  struct Object
  {
      public:
    Object(const Position& aPosition,
           double aHeight,
           double aWidth,
           double aDepth,
           double aDirection,
           double aSpeed,
           const ros::Time& aMeasurementTime,
           uint8_t aSensorId);
    /**
     * @brief getter & setter for position
     *
     * @return Position& the position strcut
     */
    Position& position();

    /**
     * @brief getter & setter for height
     *
     * @return double& the height
     */
    double& height_m();

    /**
     * @brief getter & setter for width
     *
     * @return double& the widht
     */
    double& width_m();
    /**
     * @brief getter & setter for depth
     *
     * @return double& the depth
     */
    double& depth_m();

    /**
     * @brief getter & setter for direction
     *
     * @return double& the direction
     */
    double& direction_rad();

    /**
     * @brief getter & setter for speed
     *
     * @return double& the speed
     */
    double& speed_ms();

    /**
     * @brief getter & setter for measurementTime
     *
     * @return ros::Time& the time the object was measured
     */
    ros::Time& measurementTime();
    /**
     * @brief getter & setter for sensorId
     *
     * @return uint8_t& the sensorId
     */
    uint8_t& sensorId();

      private:
    Position mPosition;         ///< @see Position.hpp for correct values
    double mHeight_m;           ///< must be between -100 and 100 meters
    double mWidth_m;            ///< must be between -100 and 100 meters
    double mDepth_m;            ///< must be between -100 and 100 meters
    double mDirecton_rad;       ///< must be between -PI and PI
    double mSpeed_ms;           ///< must be between 0 and 10 M/S
    ros::Time mMeasurementTime; ///< cannot be 0
    uint8_t mSensorId;          ///< sensorId needs to exsist
  };

} // namespace environment_controller

#endif // OBJECT_HPP