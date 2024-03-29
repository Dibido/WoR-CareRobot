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
     * @param aX
     * @param aY
     * @param aZ
     * @param aW
     */
    Rotation(double aX, double aY, double aZ, double aW);
    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double x();
    const double& x() const;

    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double y();
    const double& y() const;

    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double z();
    const double& z() const;

    /**
     * @brief Getter & setter
     *
     * @return double&
     */
    double w();
    const double& w() const;

      private:
    double mX; // mX must be between -1 and 1
    double mY; // mY must be between -1 and 1
    double mZ; // mZ must be between -1 and 1
    double mW; // mW must be between -1 and 1
  };
} // namespace environment_controller

#endif // POSITION_HPP