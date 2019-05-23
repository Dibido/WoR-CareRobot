#ifndef KINEMATICS_DENAVITHARTENBERG_HPP
#define KINEMATICS_DENAVITHARTENBERG_HPP

#include "kinematics/Configuration.hpp"
#include "kinematics/KinematicsDefines.hpp"
#include "kinematics/Link.hpp"
#include "kinematics/RobotConfiguration.hpp"
#include <array>
#include <matrix/Matrix.hpp>

/**
 * @brief Contains functions and classes to calculate end effector position of a
 * robotarm and to find the configuration to reach end effector position of a
 * robotarm
 * @author Emiel Bosman
 */
namespace kinematics
{
  /**
   * @brief Calculates End Effector position and Configuration using
   * Denavit-Hartenberg parameters
   * @author Emiel Bosman
   */
  class DenavitHartenberg
  {
      public:
    /**
     * @brief Construct a Denavit-Hartenberg object
     * @author Emiel Bosman
     */
    DenavitHartenberg();
    DenavitHartenberg(const DenavitHartenberg&) = default;
    ~DenavitHartenberg() = default;

    DenavitHartenberg& operator=(const DenavitHartenberg&) = default;

    /**
     * @brief Calculate end effector position and rotation
     * @author Emiel Bosman
     *
     * @param aBigTheta The variable to use when calculating the transformation
     * matrix for each Link
     * @param aStart Link to start with (0-indexed)
     * @param aEnd Link to end at
     * @return Matrix<double, 6, 1> The calculated end effector position
     * described in x,y and z position and yaw pitch roll rotation
     * ~~~
     * [ x y z yaw pitch roll ]
     * ~~~
     */
    Matrix<double, 6, 1> forwardKinematicsYPR(const Configuration& aBigTheta,
                                              std::size_t aStart = 0,
                                              std::size_t aEnd = 0) const;

    /**
     * @brief Run inverse kinematics to find the required Big Theta to reach a
     * given goal
     * @author Emiel Bosman
     *
     * @param aGoal Goal to reach
     * @param aCurrentBigTheta Starting position
     * @return std::array<double, cKinematicsDoF>
     */
    Configuration
        inverseKinematics(const Matrix<double, 6, 1>& aGoal,
                          const Configuration& aCurrentBigTheta) const;

      private:
    /**
     * @brief Calculate end effector position and rotation
     * @author Emiel Bosman
     *
     * @param aBigTheta The variable to use when calculating the transformation
     * matrix for each Link
     * @param aStart Link to start with (0-indexed)
     * @param aEnd Link to end at
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
    Matrix<double, 4, 4> forwardKinematics(const Configuration& aBigTheta,
                                           std::size_t aStart = 0,
                                           std::size_t aEnd = 0) const;

    /**
     * @brief Check if a configuration contains only valid values according to
     * mRobotConfiguration
     * @param aConfiguration
     * @return bool
     */
    bool isValidConfiguration(const Configuration& aConfiguration) const;

    /**
     * @brief Generate a random and valid configuration based on
     * mRobotConfiguration
     *
     * @param aConfiguration
     */
    void randomizeConfiguration(Configuration& aConfiguration) const;

    const RobotConfiguration mRobotConfiguration;
  };

} // namespace kinematics
#endif // KINEMATICS_DENAVITHARTENBERG_HPP
