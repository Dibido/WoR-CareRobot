#include "kinematics/JacobiMatrix.hpp"
#include <gtest/gtest.h>

namespace kinematics
{

  TEST(JacobiMatrix, StraightUp)
  {
    Configuration bigTheta;

    Matrix<double, 6, 7> expectedJacobian{
      { 0.0, 0.44300, 0.0, -0.127, 0.0, 0.257, 0.0 },
      { 0.088, 0.0, 0.088, 0.0, 0.088, 0.0, 0.0 },
      { 0.0, -0.088, 0.0, 0.0054999, 0.0, 0.088, 0.0 },
      { 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0 },
      { 0.0, 1.0, 0.0, -1.0, 0.0, -1.0, 0.0 },
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    };

    EXPECT_EQ(true, equals(expectedJacobian, calculateJacobiMatrix(bigTheta),
                           0.0001));
  }

  TEST(JacobiMatrix, ConfigurationOne)
  {
    Configuration bigTheta;
    bigTheta.setTheta(0, M_PI / 5);
    bigTheta.setTheta(1, M_PI / 3);
    bigTheta.setTheta(2, M_PI / 2);
    bigTheta.setTheta(3, M_PI * 1.5);
    bigTheta.setTheta(4, M_PI * 1.7);
    bigTheta.setTheta(5, M_PI / 8);
    bigTheta.setTheta(6, M_PI / 2);

    Matrix<double, 6, 7> expectedJacobian{
      { -0.40437, 0.016652, -0.1917, 0.1127, -0.14454, -0.12282, 0.0 },
      { 0.10957, 0.012098, 0.040366, 0.11043, -0.10501, 0.13282, 0.0 },
      { 0.0, -0.32633, 0.22753, 0.090119, 0.018778, -0.20264, 0.0 },
      { 1.0, 2.0816E-17, 0.59102, -0.81347, -9.7144E-17, 0.0, -0.38479 },
      { 0.0, 1.0, -1.2799E-16, -7.3899E-17, 1.0, 0.0, -0.92387 },
      { 0.0, 1.6653E-16, 0.87079, 0.50275, 0.0, 1.0, -0.040221 }
    };

    // Use epsilon of 0.00001 due to using an expected jacobian with that
    // precision
    EXPECT_EQ(true, equals(expectedJacobian, calculateJacobiMatrix(bigTheta),
                           0.00001));
  }
} // namespace kinematics