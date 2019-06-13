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
   * @param aEpsilon
   * @return true
   * @return false
   */
  inline bool doubleEquals(double lhs, double rhs, double aEpsilon)
  {
    return std::fabs(lhs - rhs) <= aEpsilon;
  }

  /**
   * @brief Checks if two radian are equal
   *
   * @pre lhs and rhs are within the range of -M_PI and M_PI
   * @param lhs
   * @param rhs
   * @param aEpsilon Amount that lhs and rhs can differ
   * @return true
   * @return false
   */
  inline bool radianEquals(double lhs, double rhs, double aEpsilon)
  {
    // Get the lowest value
    double lowest = lhs;
    double highest = rhs;
    if (lhs > rhs)
    {
      lowest = rhs;
      highest = lhs;
    }
    return doubleEquals(lowest, highest, aEpsilon) ||
           doubleEquals(lowest + M_PI * 2, highest, aEpsilon);
  }

  /**
   * @brief Checks if TransformationMatrices with both positional and rotational
   * values are equal to each other
   * Matrices are assumed to be of form : [pos, .... , pos, rad, .... , rad]
   * @author Emiel Bosman
   * @param lhs
   * @param rhs
   * @param aEpsilon_m Epsilon to use for the left positional values
   * @param aEpsilon_rad Epsilon to use for the right rotational values
   * @param aEpsilonSplit Index at which to change which epsilon to use
   * @return true
   * @return false
   */
  template <std::size_t M>
  inline bool transformationMatrixEquals(const Matrix<double, M, 1>& lhs,
                                         const Matrix<double, M, 1>& rhs,
                                         double aEpsilon_m,
                                         double aEpsilon_rad,
                                         std::size_t aEpsilonSplit)
  {
    assert(M > aEpsilonSplit);
    for (std::size_t i = 0; i < aEpsilonSplit; ++i)
    {
      if (doubleEquals(lhs[i][0], rhs[i][0], aEpsilon_m) == false)
      {
        return false;
      }
    }
    for (std::size_t i = aEpsilonSplit; i < M; ++i)
    {
      if (radianEquals(lhs[i][0], rhs[i][0], aEpsilon_rad) == false)
      {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Checks if two TransformationMatrices are equal to each other. The
   * positional values are checked with cosineSimilarity. The rotational values
   * are checked with the radianEquals function.
   * @author Brandon Geldof
   * @tparam M
   * @param lhs
   * @param rhs
   * @param aCosineSimEpsilon
   * @param aEpsilon_rad
   * @return true
   * @return false
   */
  template <std::size_t M>
  inline bool transformationMatrixCosineSim(const Matrix<double, M, 1>& lhs,
                                            const Matrix<double, M, 1>& rhs,
                                            double aCosineSimEpsilon,
                                            double aEpsilon_rad)
  {
    static_assert(M % 2 == 0, "Matrix must be able to be split in two");

    Matrix<double, M / 2, 1> lValsPos;
    Matrix<double, M / 2, 1> lRhsPos;

    for (std::size_t i = 0; i < M / 2; ++i)
    {
      lValsPos[i][0] = lhs[i][0];
      lRhsPos[i][0] = rhs[i][0];
    }
    for (std::size_t i = M / 2; i < M; ++i)
    {
      if (radianEquals(lhs[i][0], rhs[i][0], aEpsilon_rad) == false)
      {
        return false;
      }
    }

    double cCosineSimPos = cosineSimilarity(lValsPos, lRhsPos);

    return (cCosineSimPos > aCosineSimEpsilon) && (cCosineSimPos <= 1.0);
  }

  /**
   * @brief Constrain a radian between -M_PI and M_PI
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
