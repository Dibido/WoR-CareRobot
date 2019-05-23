/**
 * @file Position.hpp
 * @author Gianni Monteban
 * @brief the header file for the position data struct
 * @version 0.1
 * @date 2019-05-22
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef POSITION_HPP
#define POSITION_HPP

#include "EnvironmentConsts.hpp"
#include <stdexcept>

namespace environment_controller
{
  /**
   * @brief The data struct for a position, the structs throws an exception when
   * the value is out of range
   * @see parameters for correct values
   * @author Gianni Monteban
   */
  struct Position
  {
      public:
    Position(double aX_m, double aY_m, double aZ_m);
    /**
     * @brief Construct a new Position object
     *
     * @param aPos the object to copy
     */
    Position(const Position& aPos);
    /**
     * @brief getter & setter
     *
     * @return double& x
     */
    double& x_m();
    /**
     * @brief getter & setter
     *
     * @return double& y
     */
    double& y_m();
    /**
     * @brief getter & setter
     *
     * @return double& z
     */
    double& z_m();

      private:
    double mX_m; ///< must be between cMinRange_m and cMaxRange_m
    double mY_m; ///< must be between cMinRange_m and cMaxRange_m
    double mZ_m; ///< must be between cMinRange_m and cMaxRange_m
  };
} // namespace environment_controller

#endif // POSITION_HPP