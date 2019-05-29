#include "kinematics/BetaGenerator.hpp"
#include <algorithm>
#include <cstdint>

namespace kinematics
{
  BetaGenerator::BetaGenerator(
      const std::vector<std::pair<double, double>>& aBetaSelectors)
      : mBetaSelectors(aBetaSelectors)
  {
    std::sort(mBetaSelectors.rbegin(), mBetaSelectors.rend());
  }

  double BetaGenerator::generateBeta(
      const Matrix<double, 6, 1>& aGoal,
      const Matrix<double, 6, 1>& aCurrentPosition) const
  {
    double lSimilarity = cosineSimilarity(aGoal, aCurrentPosition);
    for (std::size_t i = 0; i < mBetaSelectors.size(); ++i)
    {
      if (lSimilarity >= mBetaSelectors[i].first)
      {
        return mBetaSelectors[i].second;
      }
    }
    return mBetaSelectors[mBetaSelectors.size() - 1].second;
  }

} // namespace kinematics
