#ifndef KINEMATICS_DENAVITHARTENBERG_HPP
#define KINEMATICS_DENAVITHARTENBERG_HPP

#include "kinematics/Joint.hpp"
#include "kinematics/Link.hpp"

#include <matrix/Matrix.hpp>
#include <vector>

namespace kinematics
{
  const double cIkBeta = 0.3;
  const double cIkEpsilon_m = 0.000001;
  const double cIkEpsilon_rad = M_PI / 180 * 5;
  const std::size_t cIkMaxIterations = 100;
  const std::size_t cDhTransformPosRadSplit = 3;

  /**
   * @brief Calculates End Effector position and Configuration using
   * Denavit-Hartenberg parameters
   */
  class DenavitHartenberg
  {
      public:
    /**
     * @brief Construct a Denavit-Hartenberg object using a set of Links
     *
     * @param lLinkConfig
     */
    explicit DenavitHartenberg(const std::vector<Link>& lLinkConfig);
    DenavitHartenberg(const DenavitHartenberg&) = default;
    ~DenavitHartenberg() = default;

    DenavitHartenberg& operator=(const DenavitHartenberg&) = default;

    Matrix<double, 6, 1>
        forwardKinematicsYPR(const std::vector<double>& bigTheta,
                             std::size_t lStart = 0,
                             std::size_t lEnd = 0) const;

    /**
     * @brief Run inverse kinematics to find the required Big Theta to reach a given goal
     * 
     * @param lGoal Goal to reach
     * @param lCurrentBigTheta Starting position
     * @return std::vector<double> 
     */
    std::vector<double> inverseKinematics(const Matrix<double, 6, 1>& lGoal, const std::vector<double>& lCurrentBigTheta) const;

      private:
    /**
     * @brief Calculate end effector position and rotation
     *
     * @param lBigTheta The variable to use when calculating the transformation matrix for each Link
     * @param lStart Link to start with (0-indexed)
     * @param lEnd Amount of links to calculate
     * @return Matrix<double, 4, 4> The calculated Forward
     * Kinematics including a 3x3 Rotational matrix and a 1x3
     * positional matrix
     * ~~~
     * [ r00 r11 r12 tx ]
     * [ r10 r21 r22 ty ]
     * [ r20 r31 r32 tz ]
     * [ 0   0   0   1  ]
     * ~~~
     */
    Matrix<double, 4, 4> forwardKinematics(const std::vector<double>& lBigTheta,
                                           std::size_t start = 0,
                                           std::size_t end = 0) const;

    /**
     * @brief Calculates the jacobian for a 7-DoF robotarm
     *
     * @param lBigTheta
     * @return Matrix<double, 6, 7>
     */
    Matrix<double, 6, 7>
        calculateJacobiMatrix(const std::vector<double>& lBigTheta) const;

    const std::vector<Link> mLinkConfiguration;
  };

} // namespace kinematics
#endif // KINEMATICS_DENAVITHARTENBERG_HPP
