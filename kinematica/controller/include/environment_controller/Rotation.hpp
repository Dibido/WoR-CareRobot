/**
 * @file Rotation.hpp
 * @author Brandon Geldof
 * @brief the header file for the rotation data struct
 * @version 0.1
 * @date 2019-06-03
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef ROTATION_HPP
#define ROTATION_HPP

#include <stdexcept>

namespace environment_controller
{
  /**
   * @brief The data struct for a position, the structs throws an exception when
   * the value is out of range
   * @author Brandon Geldof
   */
  struct Rotation
  {
      public:
    /**
     * @brief Construct a new Rotation object
     *
     * @param aRoll_rad
     * @param aPitch_rad
     * @param aYaw_rad
     * @param aQuaternion
     */
    Rotation(double aRoll_rad,
             double aPitch_rad,
             double aYaw_rad,
             double aQuaternion);
    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double roll_rad();
    const double& roll_rad() const;

    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double pitch_rad();
    const double& pitch_rad() const;

    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double yaw_rad();
    const double& yaw_rad() const;

    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double quaternion();
    const double& quaternion() const;

      private:
    double mRoll_rad;
    double mPitch_rad;
    double mYaw_rad;
    double mQuaternion;
  };
} // namespace environment_controller

#endif // POSITION_HPP