#ifndef KINEMATICS_UTILITYFUNCTIONS_HPP
#define KINEMATICS_UTILITYFUNCTIONS_HPP

#include "matrix/Matrix.hpp"
#include <cmath>

namespace kinematics
{

  /**
   * @brief Convert radians to degrees
   *
   * @param lRadian
   * @return double
   */
  inline double radian2Degrees(double lRadian)
  {
    return lRadian * 180 / M_PI;
  }

  /**
   * @brief Convert degrees to radians
   *
   * @param lDegree
   * @return double
   */
  inline double degree2Radian(double lDegree)
  {
    return M_PI / 180 * lDegree;
  }

  /**
   * @brief Checks if two double are equal to each other within a given margin
   * of error
   * @author Emiel Bosman
   *
   * @param lhs
   * @param rhs
   * @param lEpsilon
   * @return true
   * @return false
   */
  inline bool doubleEquals(double lhs, double rhs, double lEpsilon)
  {
    return std::fabs(lhs - rhs) < lEpsilon;
  }

  /**
   * @brief Checks if TransformationMatrices with both positional and rotational
   * values are equal to each other
   * Matrices are assumed to be of form : [pos, .... , pos, rad, .... , rad]
   * @author Emiel Bosman
   * @param lhs
   * @param rhs
   * @param lEpsilon_m Epsilon to use for the left positional values
   * @param lEpsilon_rad Epsilon to use for the right rotational values
   * @param lEpsilonSplit Index at which to change which epsilon to use
   * @return true
   * @return false
   */
  template <std::size_t M>
  inline bool transformationMatrixEquals(const Matrix<double, M, 1>& lhs,
                                         const Matrix<double, M, 1>& rhs,
                                         double lEpsilon_m,
                                         double lEpsilon_rad,
                                         std::size_t lEpsilonSplit)
  {
    assert(M > lEpsilonSplit);
    for (std::size_t i = 0; i < lEpsilonSplit; ++i)
    {
      if (doubleEquals(lhs[i][0], rhs[i][0], lEpsilon_m) == false)
      {
        return false;
      }
    }
    for (std::size_t i = lEpsilonSplit; i < M; ++i)
    {
      if (doubleEquals(lhs[i][0], rhs[i][0], lEpsilon_rad) == false)
      {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Constran a radian between -M_PI and M_PI
   *
   * @param aRadian
   * @return double
   */
  inline double constrainRadian(double aRadian)
  {
    aRadian = std::fmod(aRadian + M_PI, M_PI * 2);
    if (aRadian < 0)
    {
      aRadian += M_PI * 2;
    }
    aRadian -= M_PI;
    return aRadian;
  }
} // namespace kinematics
#endif // KINEMATICS_UTILITYFUNCTIONS_HPP
