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
   * Matrices are assumed to be of form : [pos, .... , pos, rad, .... , rad]
   * @param lhs
   * @param rhs
   * @param posEpsilon Epsilon to use for the left positional values
   * @param radEpsilon Epsilon to use for the right rotational values
   * @param epsilonSplit Index at which to change which epsilon to use
   * @return true
   * @return false
   */
  template <std::size_t M>
  inline bool transformationMatrixEquals(const Matrix<double, M, 1>& lhs,
                                 const Matrix<double, M, 1>& rhs,
                                 double posEpsilon,
                                 double radEpsilon,
                                 std::size_t epsilonSplit)
  {
    assert(M > epsilonSplit);
    for (std::size_t i = 0; i < epsilonSplit; ++i)
    {
      if (doubleEquals(lhs[i][0], rhs[i][0], posEpsilon) == false)
      {
        return false;
      }
    }
    for (std::size_t i = epsilonSplit; i < M; ++i)
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
