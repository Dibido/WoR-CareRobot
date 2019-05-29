#ifndef KINEMATICS_BETAGENERATOR
#define KINEMATICS_BETAGENERATOR
#include "kinematics/KinematicsDefines.hpp"
#include "matrix/Matrix.hpp"
#include <utility>
#include <vector>

namespace kinematics
{
  /**
   * @brief Generate a value (beta) based on the cosine similarity of two
   * matrices
   */
  class BetaGenerator
  {
      public:
    /**
     * @brief Construct a new Beta Generator object
     *
     * @param aBetaSelectors The cosine similarity and beta pairs to use when
     * determining which beta value to generate.
     */
    BetaGenerator(const std::vector<std::pair<double, double>>& aBetaSelectors);
    ~BetaGenerator() = default;

    /**
     * @brief Returns a beta value corresponding to the given cosine similarity
     * returns the beta related to the first selector that is smaller than the
     * cosine similarity or the beta related to the last selector if cosine
     * similarity is smaller than all values
     *
     * Examples:
     * ~~~
     * selectors:
     * - {0.8, 0.2}
     * - {0.6, 0.3}
     * - {0.4, 0.4}
     * cosine similarity: 0.9
     * returned beta: 0.2
     *
     * cosine similarity: 0.6
     * returned beta: 0.3
     *
     * cosine similarity: 0.2
     * returned beta 0.4
     * ~~~
     * @param aGoal
     * @param aCurrentPosition
     * @return double
     */
    double generateBeta(const Matrix<double, 6, 1>& aGoal,
                        const Matrix<double, 6, 1>& aCurrentPosition) const;

      private:
    std::vector<std::pair<double, double>> mBetaSelectors;
  };
} // namespace kinematics

#endif // KINEMATICS_BETAGENERATOR
