#ifndef KINEMATICS_UTILITYFUNCTIONS_HPP
#define KINEMATICS_UTILITYFUNCTIONS_HPP

#include "matrix/Matrix.hpp"
#include <cmath>

namespace kinematics
{

  inline double radian2Degrees(double lRadian)
  {
    return lRadian * 180 / M_PI;
  }

  inline double degree2Radian(double lDegree)
  {
    return M_PI / 180 * lDegree;
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
  inline bool doubleEquals(double lhs, double rhs, double lEpsilon)
  {
    return std::fabs(lhs - rhs) < lEpsilon;
  }

  /**
   * @brief Checks if TransformationMatrices with both positional and rotational
   * values are equal to each other
   * Matrices are assumed to be of form : [pos, .... , pos, rad, .... , rad]
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
} // namespace kinematics
#endif // KINEMATICS_UTILITYFUNCTIONS_HPP