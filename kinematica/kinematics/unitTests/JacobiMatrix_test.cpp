#include "kinematics/JacobiMatrix.hpp"
#include <gtest/gtest.h>

using namespace kinematics;

TEST(JacobiMatrix, StraightUp)
{
  std::vector<double> bigTheta = { 0, 0, 0, 0, 0, 0, 0 };

  Matrix<double, 6, 7> expectedJacobian{ { 0, 0.593, 0, -0.277, 0, 0.107, 0 },
                                         { 0.088, 0, 0.088, 0, 0.088, 0, 0 },
                                         { 0, -0.088, 0, 0.005499999999999991,
                                           0, 0.088, 0 },
                                         { 1, 0, 1, 0, 1, 0, -1 },
                                         { 0, 1, 0, -1, 0, -1, 0 },
                                         { 0, 0, 0, 0, 0, 0, 0 } };

  EXPECT_EQ(expectedJacobian, calculateJacobiMatrix(bigTheta));
}

TEST(JacobiMatrix, ConfigurationOne)
{
  std::vector<double> bigTheta = { M_PI/5, M_PI/3, M_PI/2, M_PI*1.5, M_PI*1.7, M_PI/8, M_PI/2};

  Matrix<double, 6, 7> expectedJacobian{
    {-0.52001, 0.062837, -0.22047, 0.22963, -0.098359, -0.077367, 0.0},
    { 0.032972, 0.045653, -0.037932, 0.15368, -0.071462, 0.094896, 0.0 },
    { 0.0, -0.33233, 0.34755, 0.15941, 0.012778, -0.064821, 0.0 },
    { 1.0, 2.0816E-17, 0.59102, -0.81347, -9.7144E-17, 0.0, -0.38479 },
    { 0.0, 1.0, -1.2799E-16, -7.3899E-17, 1.0, 0.0, -0.92387 },
    { 0.0, 1.6653E-16, 0.87079, 0.50275, 0.0, 1.0, -0.040221 }
  };

  // Use epsilon of 0.00001 due to using an expected jacobian with that precision
  EXPECT_EQ(true, equals(expectedJacobian, calculateJacobiMatrix(bigTheta), 0.00001));
}