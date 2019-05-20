#ifndef KINEMATICS_DENAVITHARTENBERG_HPP
#define KINEMATICS_DENAVITHARTENBERG_HPP

#include "kinematics/Joint.hpp"
#include "kinematics/Link.hpp"

#include <matrix/Matrix.hpp>
#include <vector>

/**
 * @brief Contains functions and classes to calculate end effector position of a
 * robotarm and to find the configuration to reach end effector position of a
 * robotarm
 * @author Emiel Bosman
 */
namespace kinematics
{
  const double cIkBeta = 0.3;
  const double cIkEpsilon_m = 0.000001;
  const double cIkEpsilon_rad = M_PI / 180 * 3;
  const std::size_t cIkMaxIterations = 1000;
  const std::size_t cDhTransformPosRadSplit = 3;

  /**
   * @brief Calculates End Effector position and Configuration using
   * Denavit-Hartenberg parameters
   * @author Emiel Bosman
   */
  class DenavitHartenberg
  {
      public:
    /**
     * @brief Construct a Denavit-Hartenberg object using a set of Links
     * @author Emiel Bosman
     *
     * @param lLinkConfig
     */
    explicit DenavitHartenberg(const std::vector<Link>& lLinkConfig);
    DenavitHartenberg(const DenavitHartenberg&) = default;
    ~DenavitHartenberg() = default;

    DenavitHartenberg& operator=(const DenavitHartenberg&) = default;

    /**
     * @brief Calculate end effector position and rotation
     * @author Emiel Bosman
     *
     * @param lBigTheta The variable to use when calculating the transformation
     * matrix for each Link
     * @param lStart Link to start with (0-indexed)
     * @param lEnd Link to end at
     * @return Matrix<double, 6, 1> The calculated end effector position
     * described in x,y and z position and yaw pitch roll rotation
     * ~~~
     * [ x y z yaw pitch roll ]
     * ~~~
     */
    Matrix<double, 6, 1>
        forwardKinematicsYPR(const std::vector<double>& bigTheta,
                             std::size_t lStart = 0,
                             std::size_t lEnd = 0) const;

    /**
     * @brief Run inverse kinematics to find the required Big Theta to reach a
     * given goal
     * @author Emiel Bosman
     *
     * @param lGoal Goal to reach
     * @param lCurrentBigTheta Starting position
     * @return std::vector<double>
     */
    std::vector<double>
        inverseKinematics(const Matrix<double, 6, 1>& lGoal,
                          const std::vector<double>& lCurrentBigTheta) const;

      private:
    /**
     * @brief Calculate end effector position and rotation
     * @author Emiel Bosman
     *
     * @param lBigTheta The variable to use when calculating the transformation
     * matrix for each Link
     * @param lStart Link to start with (0-indexed)
     * @param lEnd Link to end at
     * @return Matrix<double, 4, 4> The calculated Forward
     * Kinematics including a 3x3 Rotational matrix and a 1x3
     * positional matrix
     * ~~~
     * [ r00 r01 r02 tx ]
     * [ r10 r11 r12 ty ]
     * [ r20 r21 r22 tz ]
     * [ 0   0   0   1  ]
     * ~~~
     */
    Matrix<double, 4, 4> forwardKinematics(const std::vector<double>& lBigTheta,
                                           std::size_t lStart = 0,
                                           std::size_t lEnd = 0) const;

    const std::vector<Link> mLinkConfiguration;
  };

} // namespace kinematics
#endif // KINEMATICS_DENAVITHARTENBERG_HPP
