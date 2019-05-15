#ifndef KINEMATICS_UTILITYFUNCTIONS_HPP
#define KINEMATICS_UTILITYFUNCTIONS_HPP

#include "matrix/Matrix.hpp"
#include <cmath>

namespace kinematics
{

  inline double radian2Degrees(double radian)
  {
    return radian * 180 / M_PI;
  }

  inline double degree2Radian(double degree)
  {
    return M_PI / 180 * degree;
  }

  /**
   * @brief Checks if two double are equal to each other within a given margin
   * of error
   *
   * @param lhs
   * @param rhs
   * @param epsilon
   * @return true
   * @return false
   */
  inline bool doubleEquals(double lhs, double rhs, double epsilon)
  {
    return std::fabs(lhs - rhs) < epsilon;
  }

  /**
   * @brief Checks if TransformationMatrices with both positional and rotational
   * values are equal to each other
   *  Matrices are assumed to be of form : [pos, pos, pos, rad, rad, rad]
   * @param lhs
   * @param rhs
   * @param posEpsilon Epsilon to use for the first three positional values
   * @param radEpsilon Epsilon to use for the last three rotational values
   * @return true
   * @return false
   */
  inline bool transformYPREquals(const Matrix<double, 6, 1>& lhs,
                                 const Matrix<double, 6, 1>& rhs,
                                 double posEpsilon,
                                 double radEpsilon)
  {
    for (unsigned char i = 0; i < 3; ++i)
    {
      if (doubleEquals(lhs[i][0], rhs[i][0], posEpsilon) == false)
      {
        return false;
      }
    }
    for (unsigned char i = 3; i < 6; ++i)
    {
      if (doubleEquals(lhs[i][0], rhs[i][0], radEpsilon) == false)
      {
        return false;
      }
    }
    return true;
  }
} // namespace kinematics
#endif // KINEMATICS_UTILITYFUNCTIONS_HPP
