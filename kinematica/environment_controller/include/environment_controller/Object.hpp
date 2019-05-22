#ifndef OBJECT_HPP
#define OBJECT_HPP

#include "Position.hpp"
#include "ros/time.h"

namespace environment_controller
{
  /**
   * @brief data structure object
   * @author Gianni Monteban
   */
  struct Object
  {
      public:
    /**
     * @brief Construct a new Object object
     *
     * @param aPosition
     * @psaram aHeight
     * @param aWidth
     * @param aDepth
     * @param aDirection
     * @param aSpeed
     * @param aMeasurementTime
     * @param aSensorId
     */
    Object(const Position& aPosition,
           double aHeight,
           double aWidth,
           double aDepth,
           double aDirection,
           double aSpeed,
           const ros::Time& aMeasurementTime,
           uint8_t aSensorId);
    /**
     * @brief
     *
     * @return Position&
     */
    Position& position();

    /**
     * @brief
     *
     * @return double&
     */
    double& height_m();

    /**
     * @brief
     *
     * @return double&
     */
    double& width_m();
    /**
     * @brief
     *
     * @return double&
     */
    double& depth_m();

    /**
     * @brief
     *
     * @return double&
     */
    double& speed_ms();

    /**
     * @brief
     *
     * @return ros::Time&
     */
    ros::Time& measurementTime();
    /**
     * @brief
     *
     * @return uint8_t&
     */
    uint8_t& sensorId();

      private:
    Position mPosition;
    double mHeight_m;
    double mWidth_m;
    double mDepth_m;
    double mDirecton_rad;
    double mSpeed_ms;
    ros::Time mMeasurementTime;
    uint8_t mSensorId;
  };

} // namespace environment_controller

#endif // OBJECT_HPP