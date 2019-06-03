/**
 * @file Rotation.hpp
 * @author Brandon Geldof
 * @brief the header file for the position data struct
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
   * @see parameters for correct values
   * @author Gianni Monteban
   */
  struct Rotation
  {
      public:
    Rotation(double aRoll_rad,
             double aPitch_rad,
             double aYaw_rad,
             double aQuaternion);
    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double& roll_rad();
    const double& roll_rad() const;

    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double& pitch_rad();
    const double& pitch_rad() const;

    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double& yaw_rad();
    const double& yaw_rad() const;

    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double& quaternion();
    const double& quaternion() const;

      private:
    double mRoll_rad;  ///< must be between cMinRange_m and cMaxRange_m
    double mPitch_rad; ///< must be between cMinRange_m and cMaxRange_m
    double mYaw_rad;   ///< must be between cMinRange_m and cMaxRange_m
    double mQuaternion;
  };
} // namespace environment_controller

#endif // POSITION_HPP