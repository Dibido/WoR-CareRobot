#include "kinematics/BetaGenerator.hpp"
#include <gtest/gtest.h>

TEST(BetaGenerator, GenerateBeta)
{
  std::vector<std::pair<double, double>> betaSelectors = {
    { 0.8, 0.1 },
    { 0.7, 0.5 },
    { 0.6, 0.3 },
  };

  kinematics::BetaGenerator betaGenerator(betaSelectors);

  Matrix<double, 6, 1> lhs = { 2, 4, 5, 6, 1, 1 };
  Matrix<double, 6, 1> rhs = { 20394, 4, 5, 6760, 1, 1 };

  EXPECT_EQ(0.3, betaGenerator.generateBeta(lhs, rhs));

  rhs = { 2, 6, 4, 6, 1, 1 };

  EXPECT_EQ(0.1, betaGenerator.generateBeta(lhs, rhs));
}