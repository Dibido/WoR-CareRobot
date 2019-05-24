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
    const Position& position() const;

    /**
     * @brief getter & setter for height
     *
     * @return double& the height
     */
    double& height_m();
    const double& height_m() const;
    /**
     * @brief getter & setter for width
     *
     * @return double& the widht
     */
    double& width_m();
    const double& width_m() const;
    /**
     * @brief getter & setter for depth
     *
     * @return double& the depth
     */
    double& depth_m();
    const double& depth_m() const;
    /**
     * @brief getter & setter for direction
     *
     * @return double& the direction
     */
    double& direction_rad();
    const double& direction_rad() const;

    /**
     * @brief getter & setter for speed
     *
     * @return double& the speed
     */
    double& speed_ms();
    const double& speed_ms() const;
    /**
     * @brief getter & setter for measurementTime
     *
     * @return ros::Time& the time the object was measured
     */
    ros::Time& measurementTime();
    const ros::Time& measurementTime() const;
    /**
     * @brief getter & setter for sensorId
     *
     * @return uint8_t& the sensorId
     */
    uint8_t& sensorId();
    const uint8_t& sensorId() const;

      private:
    Position mPosition;         ///< @see Position.hpp for correct values
    double mHeight_m;           ///< must be between cMinRange_m and cMaxRange_m
    double mWidth_m;            ///< must be between cMinRange_m and cMaxRange_m
    double mDepth_m;            ///< must be between cMinRange_m and cMaxRange_m
    double mDirecton_rad;       ///< must be between cLow_rad and cHigh_rad
    double mSpeed_ms;           ///< must be between cTooSlow_ms and cTooFast_ms
    ros::Time mMeasurementTime; ///< no contraint
    uint8_t mSensorId;          ///< sensorId needs to exsist
  };

  typedef Object Obstacle;
  typedef std::vector<Obstacle> Obstacles;
} // namespace environment_controller

#endif // OBJECT_HPP