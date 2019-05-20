#include <matrix/Matrix.hpp>
#include "kinematics/UtilityFunctions.hpp"
#include <cmath>
#include <gtest/gtest.h>

TEST(UtilityFunctions, RadianDegree)
{
  double radian = M_PI_4;
  double degrees = 45.000;

  EXPECT_EQ(degrees, kinematics::radian2Degrees(radian));
  EXPECT_EQ(radian, kinematics::degree2Radian(degrees));

  double radian2 = M_PI / 180 * 20;
  double degrees2 = 20;

  EXPECT_EQ(degrees2, kinematics::radian2Degrees(radian2));
  EXPECT_EQ(radian2, kinematics::degree2Radian(degrees2));
}

TEST(UtilityFunctions, CompareFunctions)
{
  EXPECT_EQ(true, kinematics::doubleEquals(0.09, 0.089, 0.01));
  EXPECT_EQ(false, kinematics::doubleEquals(1, 2, 0.03));
  EXPECT_EQ(false, kinematics::doubleEquals(0.09, 0.1, 0.01));
}

TEST(UtilityFunctions, MatrixEquals)
{
    Matrix<double, 6, 1> lhs {2, 3, 4, 0.01, 0.03, 0.04};
    Matrix<double, 6, 1> rhs {2.05, 2.99, 4.0001, 0.010001, 0.03001, 0.03992};

    EXPECT_EQ(true, kinematics::transformationMatrixEquals(
        lhs, rhs, 0.1, 0.001, 3)
    );

    EXPECT_EQ(true, kinematics::transformationMatrixEquals(
        lhs, rhs, 0.1, 0.001, 2)
    );

    EXPECT_EQ(false, kinematics::transformationMatrixEquals(
        lhs, rhs, 0.001, 1, 3)
    );

    EXPECT_EQ(false, kinematics::transformationMatrixEquals(
        lhs, rhs, 0.1, 0.00001, 3)
    );
}