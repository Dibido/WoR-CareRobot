#ifndef KINEMATICS_BETAGENERATOR
#define KINEMATICS_BETAGENERATOR
#include "kinematics/KinematicsDefines.hpp"
#include "matrix/Matrix.hpp"
#include <utility>
#include <vector>

namespace kinematics
{
  class BetaGenerator
  {
      public:
    BetaGenerator(const std::vector<std::pair<double, double>>& aBetaSelectors);
    ~BetaGenerator() = default;

    double generateBeta(const Matrix<double, 6, 1>& aGoal,
                        const Matrix<double, 6, 1>& aCurrentPosition) const;

      private:
    const std::vector<std::pair<double, double>> mBetaSelectors;
  };
} // namespace kinematics

#endif // KINEMATICS_BETAGENERATOR
