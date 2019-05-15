#ifndef KINEMATICS_DENAVITHARTENBERG_HPP
#define KINEMATICS_DENAVITHARTENBERG_HPP

#include "kinematics/Joint.hpp"
#include "kinematics/Link.hpp"

#include <matrix/Matrix.hpp>
#include <vector>

namespace kinematics
{
  const double IK_BETA = 0.3;
  const double IK_POS_EPSILON = 0.000001;
  const double IK_RAD_EPSILON = M_PI / 180 * 5;
  const std::size_t IK_MAX_ITERATIONS = 100;
  const std::size_t DH_TRANSFORM_POS_RAD_SPLIT = 3;

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
     * @param config
     */
    explicit DenavitHartenberg(const std::vector<Link>& config);
    DenavitHartenberg(const DenavitHartenberg&) = default;
    ~DenavitHartenberg() = default;

    DenavitHartenberg& operator=(const DenavitHartenberg&) = default;

    Matrix<double, 6, 1>
        forwardKinematicsYPR(const std::vector<double>& bigTheta,
                             std::size_t start = 0,
                             std::size_t end = 0) const;

    std::vector<double> inverseKinematics(const Matrix<double, 6, 1>& goal, const std::vector<double>& currentBigTheta) const;

      private:
    /**
     * @brief Calculate forward kinematics starting and ending
     * with a given link
     *
     * @param variables Changed variables for each Link
     * @param start Link to start with (0-indexed)
     * @param end Amount of links to calculate
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
    Matrix<double, 4, 4> forwardKinematics(const std::vector<double>& bigTheta,
                                           std::size_t start = 0,
                                           std::size_t end = 0) const;

    /**
     * @brief Calculates the jacobian for a 7-DoF robotarm
     *
     * @param bigTheta
     * @return Matrix<double, 6, 7>
     */
    Matrix<double, 6, 7>
        calculateJacobiMatrix(const std::vector<double>& bigTheta) const;

    const std::vector<Link> configuration;
  };

} // namespace kinematics
#endif // KINEMATICS_DENAVITHARTENBERG_HPP
