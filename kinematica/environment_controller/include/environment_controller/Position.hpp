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
   * @brief The data struct for a position
   * @author Gianni Monteban
   */
  struct Position
  {
      public:
    /**
     * @brief Construct a new Position object
     *
     * @param aX_m
     * @param aY_m
     * @param aZ_m
     */
    Position(double aX_m, double aY_m, double aZ_m);
    /**
     * @brief Construct a new Position object
     *
     * @param aPos
     */
    Position(const Position& aPos);
    /**
     * @brief
     *
     * @return double&
     */
    double& x_m();
    /**
     * @brief
     *
     * @return double&
     */
    double& y_m();
    /**
     * @brief
     *
     * @return double&
     */
    double& z_m();

      private:
    double mX_m;
    double mY_m;
    double mZ_m;
  };
} // namespace environment_controller

#endif // POSITION_HPP