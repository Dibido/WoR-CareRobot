#ifndef KINEMATICS_DENAVITHARTENBERG_HPP
#define KINEMATICS_DENAVITHARTENBERG_HPP

#include "kinematics/Joint.hpp"
#include "kinematics/Link.hpp"

#include <matrix/Matrix.hpp>
#include <vector>

const double DEG_45 = M_PI / 4;
const double DEG_90 = M_PI / 2;
const double DEG_180 = M_PI;
const double DEG_270 = DEG_90 + DEG_180;

namespace kinematics
{

  /**
   * @brief Calculates Forward Kinematics using Denavit-Hartenberg
   * parameters
   */
  class DenavitHartenberg
  {
      public:
    /**
     * @brief Construct a new Denavit Hartenberg object using a
     * set of Links
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

    const std::vector<Link> configuration;
  };

} // namespace kinematics
#endif // KINEMATICS_DENAVITHARTENBERG_HPP
